#include <LPC17xx.H>
#include <stdint.h>
#include <stdio.h>

#define Fpclk 25e6
#define Tpwm 1e-3			   //Periódo de la señal de PWM 1ms
#define pi 3.14
#define Radio 3.3
#define Ticks_Delay 3


uint32_t T1=0, T2=0;
volatile uint32_t Flancos_Drcha=0, Flancos_Izq=0, Ticks_Drcha=0, Ticks_Izq=0;
uint32_t Velocidad_Drcha=0, Velocidad_Izq=0;
float Distancia_Drcha=0;
uint32_t vueltas=0;
uint32_t Centimetros=0;

void Giro_Derecha(void);

void Config_GPIO()
{
	LPC_GPIO1->FIODIR|=(1<<23)|(1<<25);				    //P1.23 y P1.25 como salida
	LPC_GPIO1->FIOCLR|=(1<<23)|(1<<25);		
}

void MotorD_Enable ()
{
	LPC_GPIO1->FIOSET|=(1<<23);
}

void MotorD_Disable()
{
	LPC_GPIO1->FIOCLR|=(1<<23);
}
void MotorI_Enable ()
{
	LPC_GPIO1->FIOSET|=(1<<25);
}

void MotorI_Disable()
{
	LPC_GPIO1->FIOCLR|=(1<<25);
}
//Velocidad motor
void PWM_Config()
{
	LPC_SC->PCONP|=(1<<6);				//On PWM.
	LPC_PINCON->PINSEL3|=(1<<9)|(1<<11)|(1<<17)|(1<<21);   							   
	LPC_PWM1->PR=0;				            //Fpclk=25Mhz
	LPC_PWM1->MR0=Fpclk*Tpwm;			    //Tpwm=1ms (25000)																
	LPC_PWM1->TCR=(1<<1);			        //Counter Reset
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
	NVIC_SetPriority(TIMER0_IRQn,1);			    //Asignamos prioridad 1  
  NVIC_EnableIRQ(TIMER0_IRQn);			    //Habilitamos int TIM0. 			
}

//Encoder Izquierdo.
void TIMER2_Config()
{
	LPC_SC->PCONP|=(1<<22);			    //ON TIM2.
	LPC_PINCON->PINSEL0|=(3<<8);			   //P0.4 as CAP2.0
	LPC_TIM2->PR=249999;  
  LPC_TIM2->CCR = 0x0006; 
  LPC_TIM2->TCR = 0x01;     
	NVIC_SetPriority(TIMER2_IRQn,2);    
  NVIC_EnableIRQ(TIMER2_IRQn);
}
//MAT 1.0 P1.22 PARA ADC
//flancos++;
//if(flancos==20)
//	n
//	velocidad=(1rpm/periodo)*1000*60;


void Set_Velocidad_D(int Dutty_D)
{
	uint32_t dutty=0;
	dutty=(22500-((Fpclk*Tpwm*Dutty_D)/100));				    //Calculamos ciclo de trabajo del la señal PWM motor derecho.
	LPC_PWM1->MR1=dutty;
	LPC_PWM1->MR2=22500;
	LPC_PWM1->MR2=22500;
	LPC_PWM1->MR3=dutty;
	LPC_PWM1->LER|=(1<<1)|(1<<2)|(1<<3);			         //Latch activo para MR1, MR2 y MR3
}

//Velocidad Motor Izquierdo
void Set_Velocidad_I(int Dutty_I)
{
	uint32_t dutty=0;
	dutty=(22500-((Fpclk*Tpwm*Dutty_I)/100));			   //Calculamos ciclo de trabajo de la señal PWM motor izquierdo.
	LPC_PWM1->MR4=dutty;
	LPC_PWM1->MR5=22500;
	LPC_PWM1->MR5=22500;
	LPC_PWM1->MR6=dutty;
	LPC_PWM1->LER|=(1<<4)|(1<<5)|(1<<6);
}
void TIMER0_IRQHandler()
{
	static uint32_t temp2=0;
	LPC_TIM0->IR|=(1<<5);			    //Borramos flag
	Flancos_Drcha++;			    //Contamos flancos
	Ticks_Drcha++;
	Distancia_Drcha+=((18*pi)/180)*Radio;			    //Distancia que tiene que mostrar por pantalla, distancia absoluta.
	if(Flancos_Drcha==20){
		T2=LPC_TIM0->CR1-temp2;
		vueltas++;
		Velocidad_Drcha=((1000*2*pi*3.3)/T2)*60;
		Flancos_Drcha=0;		
		temp2=LPC_TIM0->CR1;
	}
}
void TIMER2_IRQHandler()
{
	static uint32_t temp1=0;
	LPC_TIM2->IR|=(1<<4);			    //Borramos flag 
	Flancos_Izq++;
	Ticks_Izq++;
	if(Flancos_Izq==20){			                    //Si damos una vuelta entera
		T1=LPC_TIM2->CR0-temp1;
		Velocidad_Izq=((1000*2*pi*3.3)/T1)*60;			//cm/min
		Flancos_Izq=0;
		temp1=LPC_TIM2->CR0;
	}
}

void Parar()
{
	MotorD_Disable();
	MotorI_Disable();
	Set_Velocidad_D(50);			   //Motor derecho con un D del 50%
	Set_Velocidad_I(50);
}

//Avanzar
void Avanzar(int cm)
{
	uint32_t Flancos=0;
	MotorD_Enable();
	MotorI_Enable();
	Ticks_Drcha=0;
	Ticks_Izq=0;
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


void Retroceder()
{
	MotorD_Enable();
	MotorI_Enable();
	Set_Velocidad_D(25);			   //Motor derecho con un D del 25%
	Set_Velocidad_I(25);			   //Motor izquierdo con un D del 25%
}

//Giro de 90º a la derecha
void Giro_Derecha()
{
	MotorD_Enable();
	MotorI_Enable();
	Ticks_Drcha=0;
	Ticks_Izq=0;
	while((Ticks_Drcha<=10)&&(Ticks_Izq<=10)){
		Set_Velocidad_D(35);   			 //Motor derecho con un D del 25%
		Set_Velocidad_I(65);			   //Motor Izquierdo con un D del 75%
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

void Giro_Izquierda()
{
	MotorD_Enable();
	MotorI_Enable();
	Set_Velocidad_D(75);			   //Motor derecho con un D del 75%				    
	Set_Velocidad_I(25);			   //Motor izquierdo con un D del 25%
}

//Programa Principal
int main (void)
{
	NVIC_SetPriorityGrouping(2);			    //Prigroup=2
	Config_GPIO();
	PWM_Config();			    								//Configuracion PWM
	TIMER0_Config();
	TIMER2_Config();
	Avanzar(200);
	Giro_Derecha();
	while (1);
}
