/**
 * @file board_config.c
 * @brief Configuration of the microcontroller according to the board layout. 
 */

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

#include "config/board_config.h"

void board_config(void) {

    PORTA = 0;
    TRISA = 0b00000111; // Set RA0, RA1 and RA2 as input (A/D)
    PORTB = 0;
    TRISB = 0b00001110; // Set RB1, RB2 and RB3 as input (RXSM signals)
    PORTC = 0;
    TRISC = 0b11011000; // Set RC1 and RC2 as output (PWM module), RC3 and RC4 as transceiver I2C, RC6 and RC7 as transceiver COM port

    // Interrupt setup //
    IPEN = 0; // Interrupt no properties
    GIE = 1; // Global interrupts active
    PEIE = 1; // Peripheral interrupt
    TMR0IE = 1; // Timer Interrupt
    RCIE = 1; //COM receiving interrupts
    // TXIE=1; //COM transmitting interrupt (Not more used)


    // Setup TMR0 //
    T0CON = 0b10001100; //Set TMR0 at 16bit, internal clock tick on rising edge, prescaler at 1:1
    //Init TMR0H and TMR0L to have an interrupt each 1ms
    TMR0H = T0_RELOAD_HIGH;
    TMR0L = T0_RELOAD_LOW;


    /* Setup UART module */

    /*
     * Baudrate settings:
     * - 520: 9600  Kbit/s
     * - 129: 38400 Kbit/s
     * - 86 : 57600 Kbit/s
     */
    SPBRGH = 0;
    SPBRG = 129;

    /* Select 16-bit mode */
    BAUDCON = 0x8u;

    /* Configure and start peripheral */
    RCSTA = 0x80u;
    TXSTA = 0x24u;

    /* Enable Tx interrupt */
    PIE1bits.TXIE = 1;

    // Setup A/D module //
    ADCON1 = 0b00011010; // Set V3 as +REF (1V), set analog input channels ( last 0b00011100)
    ADCON2 = 0b10110101; // Set bits right shifted, acquisition time 16 Tad = 64us, conversion clock Fosc/16 = 4us (default 100 Fosc/4)

    //Setup PWM1 e PWM2 modules  //
    CCP1CON = 0b00001100; //Set CCP1 as PWM module
    PR2 = 0b11111111; //Set PWM oscillator frequency at 39,06kHz
    CCPR1L = 0; //Set dutycicle to 0%
    T2CON = 0b100; //Set TMR2 prescaler to 1
}

