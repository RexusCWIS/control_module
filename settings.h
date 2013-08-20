/*-------- REXUS Project -------*\
|*-----CWIS Microcontroller-----*|
|*----------settings.h----------*|
|*----------PIC18F2520----------*|
\*--Versione 1.0.0 - Gen. 2013--*/
;
//DEFINE//
#define SENSOR0 0b00000001 //Ingresso analogico AN0
#define SENSOR1 0b00000101 //Ingresso analogico AN1
#define SENSOR2 0b00001001 //Ingresso analogico AN2

#define LO RB3 //Lift-Off
#define SODS RB1 //Start Of Data Storage
#define SOE RB2 //Start Of Experiment

#define LASER RB4
#define HEATER CCPR1L


#define TEMPOLASER 5000 //Tempo dopo LO per accensione laser (ciclo di TMR0 =1Ms) (5000)
#define TEMPOHEATER 15000 //Tempo dopo SOE per spegnimento heater (15000)







//PROTOTIPI FUNZIONI//

void settings(void);
void main(void);
void read_ad (int n_sensor, char ad_value[6]);
void interrupt isr(void);



//PORTE//

/*
01 - MCLR *
02 - AN0  * A/D 1
03 - AN1  * A/D 2
04 - AN2  * A/D 3
05 - AN3  * VREF+ A/D
06 - RA4
07 - AN4  * A/D 4
08 - VSS  *
09 - OSC1 *
10 - OSC2 *

11 - RC0  * STEP
12 - CCP2 * DIR
13 - CCP1 * MOSFET
14 - RC3  * I2C SCL
15 - RC4  * I2C SDA
16 - RC5
17 - TX   *
18 - RX   *

19 - VSS  *
20 - VDD  *
21 - AN12 * A/D 5
22 - RB1  * SODS
23 - RB2  * SOE
24 - RB3  * LO
25 - RB4
26 - RB5
27 - RB6
28 - RB7
*/



void settings(void)
	{
	
	PORTA=0;
	TRISA=0b00000111; //Imposto RA0, RA1 e RA2 come ingressi (per convertitore A/D)
	PORTB=0;
	TRISB=0b00001111;
	PORTC=0;
	TRISC=0b11000000; //Imposto RC1 e RC2 come uscite (moduli PWM), RC6 e RC7 come ricetrasmissione seriale
	
	//SETUP modulo Interrupt//
	IPEN=0; //Disabilito la distinzione di priorità degli interrupts
	GIE=1; //Gestione globale interrupt attiva
	PEIE=1; //Interrupt di periferica attivati
	TMR0IE=1; //Interrupt di TMR0 attivato
	RCIE=1; //Interrupt di ricezione su seriale attivato
	
	
	//SETUP modulo TMR0//
	T0CON=0b11000100; //Abilito TMR0 a 8bit su sorgente clock interna con incremento su fronte di salita e prescaler a 1:32
	TMR0=100; //Imposto TMR0 in modo da avere l'overflow a 998,4Us
	
	//SETUP modulo USART//
	TXSTA=0b00100110; //Imposto trasmissione a 8bit, abilito trasmissione, imposto modalità asincrona, sync break a trasmissione completata, alta velocità
	RCSTA=0b10010000; //Abilito la porta seriale, ricezione a 8bit, abilito ricezione
	BAUDCON=0b00000010; //
	SPBRG=129; //Imposto il baudrate a 9600Kbit/s con 129 per prove, baudrate a 57600Kbit/s con 21 per RXSM
	
	//SETUP modulo A/D//	
	ADCON1=0b00011100;  //Imposto la porta RA3 come VREF+ (1V), Imposto le porte RA0, RA1 e RA2 come ingressi analogici AN0, AN1 e AN2
	ADCON2=0b10010100; //Imposto 'giustifica a destra dei Bits, Tempo di acquisizione 16 Tad = 64us, Clock di conversione Fosc/16 = 4us
		
	//SETUP moduli PWM1 e PWM2//
	PR2=0b11111111; //Imposto la frequenza dell'oscillatore PWM a 39,06kHz
	CCPR1L=10; //Imposto il Duty Cycle a 50% del modulo PWM1 (128)
	CCPR2L=0; //Imposto il Duty Cycle a 50% del modulo PWM2 (128)
	T2CON=0b100; //Abilito TMR2 e imposto il prescaler a 1
	CCP1CON=0b00001100; //Imposto modulo CCP1 come PWM
	CCP2CON=0b00001100; //Imposto modulo CCP2 come PWM
	}