#include <LPC17xx.H>
#include <stdint.h>
#include <stdio.h>
#include "uart.h" 
#include <string.h>
#include <stdlib.h>

//Constantes
#define Fpclk 25e6			                 //Frecuencia periférico
#define Tpwm 1e-3			   	               //Periódo de la señal de PWM 1ms
#define pi 3.14				
#define Radio 3.3			                   //Radio rueda
#define Ticks_Delay 3
#define Fmuestreo 1000
#define Vref 3.3				                 //Tensión de referencia para el ADC.
#define duttyforward 70			             //Ciclo de trabajo avanzar
#define duttybackward 25				         //Ciclo de trabajo retroceso
#define duttyturnrightI	75					     //Ciclo de trabajo motor izquierdo giro derecha
#define duttyturnrightD 25				       //Ciclo de trabajo motor derecho giro derecha
#define duttyturnleftD 75				         //Ciclo de trabajo motor derecho giro izquierda
#define duttyturnleftI 25			           //Ciclo de trabajo motot izquierdo giro izquierda.
#define MAX 3

//Variables
uint32_t T1=0, T2=0;				      //Periódo de los captures
volatile uint32_t Flancos_Drcha=0, Flancos_Izq=0, Ticks_Drcha=0, Ticks_Izq=0;				    //Contadores de flancos del encoder.
uint32_t Velocidad_Drcha=0, Velocidad_Izq=0;			      //Velocidad motores
float Distancia_Drcha=0;			     //Distancia recorrida por los motores.
uint32_t vueltas=0;					      //Contadores vuelta.
uint32_t Centimetros=0;
float Valor_ADC=0, Voltios=0;			    //Variables ADC 
volatile unsigned int Distancia=0;	
uint32_t i=0;
char fin=0;

char buffer[30];	// Buffer de recepción
char *ptr_rx;			// puntero de recepción
char rx_completa;// Flag de recepción de cadena completa que se activa al recibir CR(0x0D)
char *ptr_tx;			// puntero de transmisión
char tx_completa;	//Flag de transmisión

void Detector_Obstaculos(int dist);
void Parar(void);

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
																																//CAMBIAR EL 1.26
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

//Inicio de Conversión del ADC.
void TIMER1_Config()
{
	LPC_SC->PCONP|=(1<<2);					        //ON TIMER1	
	LPC_TIM1->PR=0;					                //FTIM1=Fpclk=25MHz
	LPC_TIM1->MCR=(1<<1);			  					  //Reset MAT1.0
	LPC_TIM1->MR0=(Fpclk/Fmuestreo/2);			//Han de darse 2 match para que se inicie la conversión
	LPC_TIM1->EMR=(1<<0)|(3<<4);						//Toggle
	LPC_TIM1->TCR=(1<<0);										//Counter Enable.
}	

//Sensor de distancia.
void ADC_Config()
{
	LPC_SC->PCONP|=(1<<12);			          //ON	ADC
	LPC_PINCON->PINSEL1|=(1<<14);			    //P0.23 as AD0.0 input
	LPC_PINCON->PINMODE1|=(1<<15);   			//Ni pull-up ni pull-down
	LPC_ADC->ADCR=(1<<0)|   						  //Canal 0
								(1<<8)|   			      	//CLKDIV 13MHz
								(1<<21)| 								//PDN=1
	              (6<<24);								//Inicio de conversión MAT 1.0
	LPC_ADC->ADINTEN|=(1<<8);	            //Hab. interrupción fin de conversión todos los canales, registro global	
	NVIC_EnableIRQ (ADC_IRQn);					  //Habilitacion del ADC
	NVIC_SetPriority(ADC_IRQn, 3); 			  //prioridad del ADC
}

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

void ADC_IRQHandler (void)
{
	Valor_ADC=LPC_ADC->ADSTAT;			         //borramos flag
	Valor_ADC=((LPC_ADC->ADGDR)>>4)&0xFFF;	//Leemos el valor del ADC precisión de 12 bits
	Voltios=(Valor_ADC*Vref)/4095;					         //Convertimos a voltios.
	Distancia=(int)((23.75/(Voltios-0.2))-0.42);			     //Calculamos distancia a la que está el objeto
	Detector_Obstaculos(Centimetros);
}

//Parar motor
void Parar()
{
	MotorD_Disable();				        //Deshabilitamos motor derecho
	MotorI_Disable();				        //Deshabilitamos motor izquierdo
	Set_Velocidad_D(50);			      //Motor derecho con un D del 50%
	Set_Velocidad_I(50);				    //Motor izquierdo con un D del 50%
}

void Detector_Obstaculos(int Dist)
{
	static int i=0;
	
	if(Distancia<=Dist){
		i++;
		while(i==MAX)
			Parar();
	}else{
		i--;
		if(i < 0){
			i=0;
		}
	}
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
	Set_Velocidad_D(duttyforward);			   //Motor derecho con un D del 75%
	Set_Velocidad_I(duttyforward);   			 //Motor izquierdo con un D del 75%
	do{
		while(Ticks_Izq==Ticks_Drcha){
			Set_Velocidad_D(duttyforward);			   //Motor derecho con un D del 75%
			Set_Velocidad_I(duttyforward);   			 //Motor izquierdo con un D del 75%
		}
			while(Ticks_Izq>Ticks_Drcha){
			Set_Velocidad_I(50);
			Set_Velocidad_D(duttyforward);
		}
		while(Ticks_Drcha>Ticks_Izq){
			Set_Velocidad_D(50);
			Set_Velocidad_I(duttyforward);
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
			Set_Velocidad_D(duttybackward);			   //Motor derecho con un D del 25%
			Set_Velocidad_I(duttybackward);   			 //Motor izquierdo con un D del 25%
		}
			while(Ticks_Izq>Ticks_Drcha){
			Set_Velocidad_I(50);
			Set_Velocidad_D(duttybackward);
		}
		while(Ticks_Drcha>Ticks_Izq){
			Set_Velocidad_D(50);
			Set_Velocidad_I(duttybackward);
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
		Set_Velocidad_D(duttyturnrightD);   			 //Motor derecho con un D del 35% (Bajamos velocidad para evitar derrapes en el giro)
		Set_Velocidad_I(duttyturnrightI);			   //Motor Izquierdo con un D del 65%
	}
	while(Ticks_Drcha<=10){
		Set_Velocidad_I(50);
		Set_Velocidad_D(duttyturnrightD);
	}
	while(Ticks_Izq<=10){
		Set_Velocidad_I(duttyturnrightI);
		Set_Velocidad_D(50);
	}
	Avanzar(Centimetros);
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
		Set_Velocidad_D(duttyturnleftD);   			 //Motor derecho con un D del 65%
		Set_Velocidad_I(duttyturnleftI);			   //Motor Izquierdo con un D del 35%
	}
	while(Ticks_Drcha<=10){
		Set_Velocidad_I(50);
		Set_Velocidad_D(duttyturnleftD);
	}
	while(Ticks_Izq<=10){
		Set_Velocidad_I(duttyturnleftI);
		Set_Velocidad_D(50);
	}
	Avanzar(Centimetros);
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
	TIMER1_Config();				     					//Configuración TIM1.
	ADC_Config();													//Configuración ADC.
	uart0_init(9600);				              //Uart a 9600 baudios
	ptr_rx=buffer;
	
	tx_cadena_UART0("Introduzca la sentencia de movimientos, puede introducir hasta 10 movimientos:\n");			//Mandamos mensaje
	while(tx_completa==0);			    //Espera a que se termine de mandar la cadena de caracteres. Cuando termina tx_completa=1
	tx_completa=0;			     //Borrar flag de transmisión
	
	do{
		if(rx_completa){			    //Si se ha llenado el buffer de recepción (rx_completa=1)
			rx_completa=0;
			for(i=0; i<30; i+=3){
				if(buffer[i]=='A'){
					Centimetros=atoi(&buffer[i+1]);	
					Avanzar(Centimetros);
				}
				else if(buffer[i]=='D'){
					Centimetros=atoi(&buffer[i+1]);
					Giro_Derecha();
					//Avanzar(Centimetros);
				}
				else if(buffer[i]=='I'){
					Centimetros=atoi(&buffer[i+1]);
					Giro_Izquierda();
					//Avanzar(Centimetros);
				}
				else if(buffer[i]=='R'){					
					Centimetros=atoi(&buffer[i+1]);
					Retroceder(Centimetros);
				}
				else if(buffer[i]=='O'){
					Centimetros=atoi(&buffer[i+1]);
					Detector_Obstaculos(Centimetros);
				}
				else if(buffer[i]==0x0D){
					fin=1;
					break;
				}
				else{
					tx_cadena_UART0("Comando erroneo\n\r");
					fin=1;
					break;
				}
			}	
		}
	}while(fin==0);
	while (1);
}
