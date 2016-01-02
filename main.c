#include <LPC17xx.H>
#include <stdint.h>
#include <stdio.h>


//Constantes
#define Fpclk 25e6			   //Frecuencia periférico
#define Tpwm 1e-3			   	 //Periódo de la señal de PWM 1ms
#define pi 3.14				
#define Radio 3.3			     //Radio rueda
#define Ticks_Delay 3

//Variables
uint32_t T1=0, T2=0;
volatile uint32_t Flancos_Drcha=0, Flancos_Izq=0, Ticks_Drcha=0, Ticks_Izq=0;				    //Contadores de flancos del encoder.
uint32_t Velocidad_Drcha=0, Velocidad_Izq=0;
float Distancia_Drcha=0;			     //Distancia recorrida por los motores.
uint32_t vueltas=0;
uint32_t Centimetros=0;

//Configuracion GPIOS enable motores
void Config_GPIO()
{
	LPC_GPIO1->FIODIR|=(1<<23)|(1<<25);				    //P1.23 y P1.25 como salida
	LPC_GPIO1->FIOCLR|=(1<<23)|(1<<25);				    //P1.23 y P1.25 inicialmente a 0. Enable activo a nivel alto.
}

//Habilitación motor derecho, P1.23
void MotorD_Enable ()
{
	LPC_GPIO1->FIOSET|=(1<<23);			   //P1.23 a 1
}

//Deshabilitación motor derecho
void MotorD_Disable()
{
	LPC_GPIO1->FIOCLR|=(1<<23);			    //P1.23 a 0
}

//Habilitación motor izquierdo, P1.25
void MotorI_Enable ()
{
	LPC_GPIO1->FIOSET|=(1<<25);			    //P1.25 a 1
}

//Deshabilitación motor izquierdo.
void MotorI_Disable()
{
	LPC_GPIO1->FIOCLR|=(1<<25);
}
//Velocidad motor
void PWM_Config()
{
	LPC_SC->PCONP|=(1<<6);				                                //On PWM.
	LPC_PINCON->PINSEL3|=(1<<9)|(1<<11)|(1<<17)|(1<<21);			    //P1.20 as PWM1.2, P1.21 as PWM1.3, P1.24 as PWM1.5, P1.26 as PWM1.6  							   
	LPC_PWM1->PR=0;				                                        //Fpclk=25Mhz
	LPC_PWM1->MR0=Fpclk*Tpwm;			                                //Tpwm=1ms (25000)																
	LPC_PWM1->TCR=(1<<1);			                                    //Counter Reset
	LPC_PWM1->LER=(1<<0);			   						//Latch activo para MR0
	LPC_PWM1->PCR|=(1<<2)|(1<<3)|(1<<5)|(1<<6)|(1<<10)|(1<<11)|(1<<13)|(1<<14);
	LPC_PWM1->MCR=(1<<1);
	LPC_PWM1->TCR=(1<<0)|(1<<3);				    //Enable PWM, Enable Counter.
}

//Encoder derecho.
void TIMER0_Config()
{
	LPC_SC->PCONP|=(1<<1);					             //ON TIM0.
	LPC_PINCON->PINSEL3|=(1<<22)|(1<<23);			   //P1.27 as CAP0.1
	LPC_TIM0->PR=249999;			     							 //PCLK=25e6/249999+1
  LPC_TIM0->CCR|=(1<<4)|(1<<5); 			       	 //Flanco de bajada, Interrupcion CAP0.1  
	LPC_TIM0->TCR = 0x01;			                   //Start Timer.  	
	NVIC_SetPriority(TIMER0_IRQn,1);			       //Asignamos prioridad 1  
  NVIC_EnableIRQ(TIMER0_IRQn);			    			 //Habilitamos int TIM0. 			
}

//Encoder Izquierdo.
void TIMER2_Config()
{
	LPC_SC->PCONP|=(1<<22);			    		 //ON TIM2.
	LPC_PINCON->PINSEL0|=(3<<8);			    //P0.4 as CAP2.0
	LPC_TIM2->PR=249999;			    			  //PCLK=25e6/249999+1
	LPC_TIM2->CCR|=(1<<1)|(1<<2);			    //Flanco de bajada, Interrupción CAP2.0
  //LPC_TIM2->CCR = 0x0006; 
  LPC_TIM2->TCR = 0x01;				    //Start Timer.    
	NVIC_SetPriority(TIMER2_IRQn,2);    
  NVIC_EnableIRQ(TIMER2_IRQn);
}
//MAT 1.0 P1.22 PARA ADC
//flancos++;
//if(flancos==20)
//	n
//	velocidad=(1rpm/periodo)*1000*60;

//Velocidad Motor Derecho.
void Set_Velocidad_D(int Dutty_D)
{
	uint32_t dutty=0;			    												  //Variable para calcular el ciclo de trabajo
	dutty=(22500-((Fpclk*Tpwm*Dutty_D)/100));				    //Calculamos ciclo de trabajo del la señal PWM motor derecho.
	LPC_PWM1->MR1=dutty;			                          //Set 
	LPC_PWM1->MR2=22500;				     				            //Reset
	LPC_PWM1->MR2=22500;					    //Reset
	LPC_PWM1->MR3=dutty;
	LPC_PWM1->LER|=(1<<1)|(1<<2)|(1<<3);			         //Latch activo para MR1, MR2 y MR3
}

//Velocidad Motor Izquierdo
void Set_Velocidad_I(int Dutty_I)
{
	uint32_t dutty=0;					      //Variable para calcular el ciclo de trabajo					
	dutty=(22500-((Fpclk*Tpwm*Dutty_I)/100));			   //Calculamos ciclo de trabajo de la señal PWM motor izquierdo.
	LPC_PWM1->MR4=dutty;				     //Set
	LPC_PWM1->MR5=22500;			     //Reset
	LPC_PWM1->MR5=22500;
	LPC_PWM1->MR6=dutty;
	LPC_PWM1->LER|=(1<<4)|(1<<5)|(1<<6);
}

//IRQ TIM0
void TIMER0_IRQHandler()
{
	static uint32_t temp2=0;
	LPC_TIM0->IR|=(1<<5);			    //Borramos flag
	Flancos_Drcha++;			        //Contamos flancos para calcular vueltas
	Ticks_Drcha++;				        //Contamos flancos
	Distancia_Drcha+=((18*pi)/180)*Radio;			    //Distancia que tiene que mostrar por pantalla, distancia absoluta.
	if(Flancos_Drcha==20){			    //Si damos una vuelta   						
		T2=LPC_TIM0->CR1-temp2;
		vueltas++;
		Velocidad_Drcha=((1000*2*pi*3.3)/T2)*60;
		Flancos_Drcha=0;		
		temp2=LPC_TIM0->CR1;
	}
}

//IRQ TIM2
void TIMER2_IRQHandler()
{
	static uint32_t temp1=0;
	LPC_TIM2->IR|=(1<<4);			    //Borramos flag 
	Flancos_Izq++;    					  //Contamos flancos para calcular vueltas
	Ticks_Izq++;				          //Contamos flancos
	if(Flancos_Izq==20){			                    //Si damos una vuelta entera
		T1=LPC_TIM2->CR0-temp1;
		Velocidad_Izq=((1000*2*pi*3.3)/T1)*60;			//cm/min
		Flancos_Izq=0;				                      //Reseteamos la variable flancos
		temp1=LPC_TIM2->CR0;
	}
}

//Parar motor
void Parar()
{
	MotorD_Disable();				        //Deshabilitamos motor derecho
	MotorI_Disable();				        //Deshabilitamos motor izquierdo
	Set_Velocidad_D(50);			      //Motor derecho con un D del 50%
	Set_Velocidad_I(50);				    //Motor izquierdo con un D del 50%
}

//Avanzar
void Avanzar(int cm)
{
	uint32_t Flancos=0;
	MotorD_Enable();					     //Habilitamos motor derecho
	MotorI_Enable();				       //Habilitamos motor Izquierdo
	Ticks_Drcha=0;				         //Inicializamos los ticks del encoder derecho
	Ticks_Izq=0;    					//Inicializamos los ticks del encoder izquierdo
	Flancos=((20*cm)/21);
	Set_Velocidad_D(75);			   //Motor derecho con un D del 75%
	Set_Velocidad_I(75);   			 //Motor izquierdo con un D del 75%
	do{
		while(Ticks_Izq==Ticks_Drcha){
			Set_Velocidad_D(75);			   //Motor derecho con un D del 75%
			Set_Velocidad_I(75);   			 //Motor izquierdo con un D del 75%
		}
			while(Ticks_Izq>Ticks_Drcha){
			Set_Velocidad_I(50);
			Set_Velocidad_D(75);
		}
		while(Ticks_Drcha>Ticks_Izq){
			Set_Velocidad_D(50);
			Set_Velocidad_I(75);
		}
	}while((Ticks_Drcha<=Flancos)&&(Ticks_Izq<=Flancos));
	Parar();
}

//Retroceder
void Retroceder(int cm)
{
	uint32_t Flancos=0;
	MotorD_Enable();
	MotorI_Enable();
	Ticks_Drcha=0;				         //Inicializamos los ticks del encoder derecho
	Ticks_Izq=0;    					//Inicializamos los ticks del encoder izquierdo
	Flancos=((20*cm)/21);
	Set_Velocidad_D(25);			   //Motor derecho con un D del 25%
	Set_Velocidad_I(25);			   //Motor izquierdo con un D del 25%
		//Control trayectoria recta
		do{
		while(Ticks_Izq==Ticks_Drcha){				     //Mientras los flancos de los dos encoders sean iguales
			Set_Velocidad_D(25);			   //Motor derecho con un D del 25%
			Set_Velocidad_I(25);   			 //Motor izquierdo con un D del 25%
		}
			while(Ticks_Izq>Ticks_Drcha){
			Set_Velocidad_I(50);
			Set_Velocidad_D(25);
		}
		while(Ticks_Drcha>Ticks_Izq){
			Set_Velocidad_D(50);
			Set_Velocidad_I(25);
		}
	}while((Ticks_Drcha<=Flancos)&&(Ticks_Izq<=Flancos));
	Parar();
}

//Giro de 90º a la derecha
void Giro_Derecha()
{
	MotorD_Enable();			      //Habilitamos motor derecho.
	MotorI_Enable();				    //Habilitamos motor izquierdo.
	Ticks_Drcha=0;				      //Inicializamos flancos del encoder derecho
	Ticks_Izq=0;				        //Inicializamos flancos del encoder izquierdo
	
	while((Ticks_Drcha<=10)&&(Ticks_Izq<=10)){
		Set_Velocidad_D(35);   			 //Motor derecho con un D del 35% (Bajamos velocidad para evitar derrapes en el giro)
		Set_Velocidad_I(65);			   //Motor Izquierdo con un D del 65%
	}
	while(Ticks_Drcha<=10){
		Set_Velocidad_I(50);
		Set_Velocidad_D(35);
	}
	while(Ticks_Izq<=10){
		Set_Velocidad_I(65);
		Set_Velocidad_D(50);
	}
	Parar();
}

//Giro de 90º a la izquierda
void Giro_Izquierda()
{
	MotorD_Enable();
	MotorI_Enable();
	Ticks_Drcha=0;				      //Inicializamos flancos del encoder derecho
	Ticks_Izq=0;				        //Inicializamos flancos del encoder izquierdo
	
	//Control giro de 90º, para un giro de 90º tienen que pasar 10 flancos.
	while((Ticks_Drcha<=10)&&(Ticks_Izq<=10)){			     //Mientras los flancos de ambos encoders sean menores o igual a 10
		Set_Velocidad_D(65);   			 //Motor derecho con un D del 65%
		Set_Velocidad_I(35);			   //Motor Izquierdo con un D del 35%
	}
	while(Ticks_Izq<=10){
		Set_Velocidad_I(35);
		Set_Velocidad_D(50);
	}
	while(Ticks_Drcha<=10){
		Set_Velocidad_I(50);
		Set_Velocidad_D(65);
	}
	Parar();
}

//Programa Principal
int main (void)
{
	NVIC_SetPriorityGrouping(2);			    //Prigroup=2
	Config_GPIO();			   							  //Configuración GPIOS
	PWM_Config();			    								//Configuracion PWM
	TIMER0_Config();				     					//Configuración TIM0.
	TIMER2_Config();				    			    //Configuración TIM2.
	Avanzar(100);
	Giro_Derecha();
	Retroceder(50);
	Giro_Izquierda();
	while (1);
}
