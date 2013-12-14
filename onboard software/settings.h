/*-------- REXUS Project -------*\
|*-----CWIS Microcontroller-----*|
|*----------settings.h----------*|
|*----------PIC18F2520----------*|
\*--Versione 1.0.0 - Gen. 2013--*/
;
// DEFINE //
#define SENSOR0 0b00000001 // Analog input AN0 
#define SENSOR1 0b00000101 // Analog input AN1
#define SENSOR2 0b00001001 // Analog input AN2

#define LO       RB3  // Lift-Off
#define SODS     RB1  // Start Of Data Storage
#define SOE      RB2  // Start Of Experiment

#define LO_LED   RB5  // Lift-Off LED
#define SODS_LED RB6  // Start Of Data Storage LED
#define SOE_LED  RB7  // Start Of Experiment LED

#define LASER RB4
#define HEATER CCPR1L

// 1 unit = 1ms //
#define TEMPOLASER 5      // Timer for Laser Power On after LO (default 5000)
#define TEMPOHEATER 15000 //Timer for Heater Power Off after predefined time (default 15000)
#define TEMPOCONV 10      //Timer between ADC conversion (default 100)
#define TEMPOAB 10        //Timer for debounce system (default 50)

// I2C macros //

#define T0_RELOAD_HIGH  0xFDu
#define T0_RELOAD_LOW   0x8Eu

#define I2C_RX_FRAME_SIZE   32u
#define I2C_TX_FRAME_SIZE   32u

#define I2C_ADDRESS 0x22u

//FUNCTION PROTOTYPES//

void settings(void);
void main(void);
void read_ad (int, char);
void interrupt isr(void);
void DelayMillisec(int);
void sendtemp(int);
unsigned int bin_dec(unsigned int bin);



// PORTS schematic //

/*
01 - MCLR *
02 - AN0  * A/D 1 CELL TEMP SENSOR
03 - AN1  * A/D 2 HEATER TEMP SENSOR
04 - AN2  * A/D 3 OTHER TEMP SENSOR
05 - AN3  * VREF+ A/D
06 - RA4  
07 - AN4  * A/D 4 PRESSURE SENSOR
08 - VSS  * GND
09 - OSC1 * XTAL
10 - OSC2 * XTAL

11 - RC0  * LO LED // ### temp0
12 - RC1  * SODS LED // ### temp1
13 - CCP1 * HEATER DRIVER
14 - RC3  * I2C SCL
15 - RC4  * I2C SDA
16 - RC5  * SOE LED
17 - TX   * RS-488
18 - RX   * RS-488

19 - VSS  * GND
20 - VDD  * 5V
21 - RB0
22 - RB1  * SODS
23 - RB2  * SOE
24 - RB3  * LO
25 - RB4  * LASER DRIVER
26 - RB5
27 - RB6  * PGC
28 - RB7  * PGD
*/



void settings(void)
	{
	
	PORTA=0;
	TRISA=0b00000111; // Setting RA0, RA1 and RA2 as input (A/D)
	PORTB=0;
	TRISB=0b00001111;
	PORTC=0;
	TRISC=0b11011000; // Setting RC1 and RC2 as output (PWM module), RC3 e RC4 as transceiver I2C, RC6 e RC7 as transceiver COM port
	
	// Interrupt setup //
	IPEN=0; // Interrupt no properties
	GIE=1; // Global interrupts active
	PEIE=1; // Peripheral interrupt
	TMR0IE=1; // Timer Interrupt
	RCIE=1; //COM receiving interrups
	// TXIE=1; //COM transmitting interrupt (Not more used)
	
	
	// Setup tmr0 // Setted in code
	//T0CON=0b11000100; //Set TMR0 at 8bit, internal clock tick on rising edge, prescaler at 1:32
	//TMR0=100; //Set TMR0 to have an l'overflow at a 998,4Us
	
	// Setup USART module //
	TXSTA=0b00100110; // Set 8 bit transmission, async mode, sync break at ended transmission, high speed
	RCSTA=0b10010000; //Set COM port, 8bit receiving, 
	BAUDCON=0b00001010; //
	SPBRG=129; //Set the baudrate to 9600Kbit/s with 520, baudrate to 57600Kbit/s with 86, baudrate to 38400Kbit/s with 129
	
	
	// Setup A/D module //	
	ADCON1=0b00011010;  // Set V3 as +REF (1V), Set digital input to analog input ( era 0b00011100)
	ADCON2=0b10110101; // Set bits right shifted, acquisition time 16 Tad = 64us (prima 4Tad), conversion clock Fosc/16 = 4us (default 100 Fosc/4)
		
	//Setup PWM1 e PWM2 modules  //
	PR2=0b11111111; //Set oscillator frequency at PWM a 39,06kHz
	CCPR1L=0; //Set dutycicle to 50% of the PWM1 (128)
	T2CON=0b100; //Set TMR2 set prescaler to 1
	CCP1CON=0b00001100; //Set CCP1 as PWM module
	}