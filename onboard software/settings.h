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

#define LO       RB3  //Lift-Off
#define SODS     RB1  //Start Of Data Storage
#define SOE      RB2  //Start Of Experiment
#define LO_LED   RB5  //Lift-Off LED
#define SODS_LED RB6  //Start Of Data Storage LED
#define SOE_LED  RB7  //Start Of Experiment LED

#define LASER RB4
#define HEATER CCPR1L

// 1 unit = 1ms //
#define TEMPOLASER 5      //Tempo dopo LO per accensione laser (5000)
#define TEMPOHEATER 15000 //Tempo dopo SOE per spegnimento heater (15000)
#define TEMPOCONV 10       //Tempo tra conversioni ADC (100)
#define TEMPOAB 10        //Tempo anti-bump segnali RXSM (50)
#define TLINEARE 2        //Temperatura entro la quale effettuare la regolazione lineare del PWM (1°grado) 
#define DIMAVG 1          //Conversion average values
#define TR_HEATER 10      //Tempo refresh duty cycle heater
  






//PROTOTIPI FUNZIONI//

void settings(void);
void main(void);
void read_ad (int, char);
void interrupt isr(void);
void DelayMillisec(int);
void sendtemp(int);
unsigned int bin_dec(unsigned int bin);



//PORTE//

/*
01 - MCLR *
02 - AN0  * A/D 1 CELL TEMP SENSOR
03 - AN1  * A/D 2 HEATER TEMP SENSOR
04 - AN2  * A/D 3 OTHER TEMP SENSOR
05 - AN3  * VREF+ A/D
06 - RA4  ### led sods
07 - AN4  * A/D 4 PRESSURE SENSOR
08 - VSS  * GND
09 - OSC1 * XTAL
10 - OSC2 * XTAL

11 - RC0  ### temp0
12 - RC1  ### temp1
13 - CCP1 * HEATER DRIVER
14 - RC3  * I2C SCL
15 - RC4  * I2C SDA
16 - RC5
17 - TX   * RS-488
18 - RX   * RS-488

19 - VSS  * GND
20 - VDD  * 5V
21 - RB0
22 - RB1  * SODS
23 - RB2  * SOE
24 - RB3  * LO
25 - RB4  * LASER DRIVER
26 - RB5  * LO LED
27 - RB6  * SODS LED
28 - RB7  * SOE LED
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
	//TXIE=1; //Interrupt di trasmissione su seriale attivato (Non più usato)
	
	
	//SETUP modulo TMR0//
	T0CON=0b11000100; //Abilito TMR0 a 8bit su sorgente clock interna con incremento su fronte di salita e prescaler a 1:32
	TMR0=100; //Imposto TMR0 in modo da avere l'overflow a 998,4Us
	
	//SETUP modulo USART//
	TXSTA=0b00100110; //Imposto trasmissione a 8bit, abilito trasmissione, imposto modalità asincrona, sync break a trasmissione completata, alta velocità
	RCSTA=0b10010000; //Abilito la porta seriale, ricezione a 8bit, abilito ricezione
	BAUDCON=0b00001010; //
	SPBRG=129; //Imposto il baudrate a 9600Kbit/s con 520, baudrate a 57600Kbit/s con 86, baudrate a 38400Kbit/s con 129
	
	
	//SETUP modulo A/D//	
	ADCON1=0b00011100;  //Imposto la porta RA3 come VREF+ (1V), Imposto le porte RA0, RA1 e RA2 come ingressi analogici AN0, AN1 e AN2
	ADCON2=0b10110101; //Imposto 'giustifica a destra dei Bits, Tempo di acquisizione 16 Tad = 64us (prima 4Tad), Clock di conversione Fosc/16 = 4us (prima 100 Fosc/4)
		
	//SETUP moduli PWM1 e PWM2//
	PR2=0b11111111; //Imposto la frequenza dell'oscillatore PWM a 39,06kHz
	CCPR1L=0; //Imposto il Duty Cycle a 50% del modulo PWM1 (128)
	//CCPR2L=0; //Imposto il Duty Cycle a 50% del modulo PWM2 (128) (Non più usato)
	T2CON=0b100; //Abilito TMR2 e imposto il prescaler a 1
	CCP1CON=0b00001100; //Imposto modulo CCP1 come PWM
	//CCP2CON=0b00001100; //Imposto modulo CCP2 come PWM (Non più usato)
	}