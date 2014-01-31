/*-------- REXUS Project -------*\
|*-----CWIS Microcontroller-----*|
|*----------PIC18F2520----------*|
\*-Fabrizio Mancino - Gen. 2013-*/

#ifndef DEF_BOARD_CONFIG_H
#define DEF_BOARD_CONFIG_H

#include <htc.h>

// General macros //
#define SENSOR0 0b00000001 // Analog input AN0 
#define SENSOR1 0b00000101 // Analog input AN1
#define SENSOR2 0b00001001 // Analog input AN2

#define LO       PORTBbits.RB3  // Lift-Off
#define SODS     PORTBbits.RB1  // Start Of Data Storage
#define SOE      PORTBbits.RB2  // Start Of Experiment

#define LO_LED   PORTBbits.RB5  // Lift-Off LED
#define SODS_LED PORTBbits.RB6  // Start Of Data Storage LED
#define SOE_LED  PORTBbits.RB7  // Start Of Experiment LED

#define LASER   PORTBbits.RB4     // Laser command output
#define HEATER  CCPR1L // Heater command output (PWM)

/** @brief Temperature control setpoint. */
#define TEMPERATURE_CONTROL_SETPOINT    0x273u // 40°C
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
#define TIME_LASER_ON    5000   // Timer for Laser Power On after LO (default 5000)
#define TIME_HEATER_OFF  20000  //Timer for Heater Power Off after predefined time (default 15000)
#define RFH_HEATER  5           //Timer for refresh heater power (default X)
#define RFH_ADC     100         //Timer for refresh ADC (default 100)
#define TIME_DEBOUNCE    50     //Timer for debounce system (default 50)
#define TIME_ACQUISITION_OFF    140000  //Timer for stop the camera acquisition (default 140000)

//FUNCTION PROTOTYPES//
void board_config(void);
void sendtemp(int);

#endif  /* DEF_BOARD_CONFIG_H */

