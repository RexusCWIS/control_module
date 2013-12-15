/*-------- REXUS Project -------*\
|*-----CWIS Microcontroller-----*|
|*----------PIC18F2520----------*|
\*-Fabrizio Mancino - Gen. 2013-*/
;
// General macros //
#define SENSOR0 0b00000001 // Analog input AN0 
#define SENSOR1 0b00000101 // Analog input AN1
#define SENSOR2 0b00001001 // Analog input AN2

#define LO       RB3  // Lift-Off
#define SODS     RB1  // Start Of Data Storage
#define SOE      RB2  // Start Of Experiment

#define LO_LED   RB5  // Lift-Off LED
#define SODS_LED RB6  // Start Of Data Storage LED
#define SOE_LED  RB7  // Start Of Experiment LED

#define LASER RB4     // Laser command output
#define HEATER CCPR1L // Heater command output (PWM)

/** @brief Temperature control setpoint. */
#define TEMPERATURE_CONTROL_SETPOINT    0x100u
/** @brief Temperature control proportional gain. */
#define TEMPERATURE_CONTROL_PGAIN    10u

/* TMR0 (1ms tick) reload values */
#define T0_RELOAD_HIGH 0xEC //( last 0xFDu)
#define T0_RELOAD_LOW  0x77 // (last 0x8Eu)

/* I2C macros */
#define I2C_RX_FRAME_SIZE   32u
#define I2C_TX_FRAME_SIZE   32u
#define I2C_ADDRESS 0x22u

// 1 unit = 1ms //
#define TEMPOLASER 5      // Timer for Laser Power On after LO (default 5000)
#define TEMPOHEATER 15000 //Timer for Heater Power Off after predefined time (default 15000)
#define TEMPOCONV 250      //Timer between ADC conversion (default 100)
#define TEMPOAB 50        //Timer for debounce system (default 50)

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



void settings(void) {
	
	PORTA=0;
	TRISA=0b00000111; // Set RA0, RA1 and RA2 as input (A/D)
	PORTB=0;
	TRISB=0b00001110; // Set RB1, RB2 and RB3 as input (RXSM signals)
	PORTC=0;
	TRISC=0b11011000; // Set RC1 and RC2 as output (PWM module), RC3 and RC4 as transceiver I2C, RC6 and RC7 as transceiver COM port
	
	// Interrupt setup //
	IPEN=0; // Interrupt no properties
	GIE=1; // Global interrupts active
	PEIE=1; // Peripheral interrupt
	TMR0IE=1; // Timer Interrupt
	RCIE=1; //COM receiving interrupts
	// TXIE=1; //COM transmitting interrupt (Not more used)
	
	
	// Setup TMR0 //
	T0CON=0b10001100; //Set TMR0 at 16bit, internal clock tick on rising edge, prescaler at 1:1
	//Init TMR0H and TMR0L to have an interrupt each 1ms
	TMR0H=T0_RELOAD_HIGH;
	TMR0L=T0_RELOAD_LOW;
	
	
	// Setup USART module //
	TXSTA=0b00100110; // Set 8 bit transmission, not sync mode, sync break at ended transmission, high speed
	RCSTA=0b10010000; // Set COM port, 8bit receiving mode
	BAUDCON=0b00001010; // Baudrate control
	SPBRG=129; //Set the baudrate to 9600Kbit/s with 520, baudrate to 57600Kbit/s with 86, baudrate to 38400Kbit/s with 129
	
	
	// Setup A/D module //	
	ADCON1=0b00011010;  // Set V3 as +REF (1V), set analog input channels ( last 0b00011100)
	ADCON2=0b10110101; // Set bits right shifted, acquisition time 16 Tad = 64us, conversion clock Fosc/16 = 4us (default 100 Fosc/4)
	
	//Setup PWM1 e PWM2 modules  //
	CCP1CON=0b00001100; //Set CCP1 as PWM module
	PR2=0b11111111; //Set PWM oscillator frequency at 39,06kHz
	CCPR1L=0; //Set dutycicle to 0%
	T2CON=0b100; //Set TMR2 prescaler to 1
}
