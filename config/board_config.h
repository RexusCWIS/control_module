/*-------- REXUS Project -------*\
|*-----CWIS Microcontroller-----*|
|*----------PIC18F2520----------*|
\*-Fabrizio Mancino - Gen. 2013-*/

#ifndef DEF_BOARD_CONFIG_H
#define DEF_BOARD_CONFIG_H

#include <htc.h>

/* I/O macros */
#define ADC_CONVERSION  ADCON0bits.GO

#define TEMPERATURE_SENSOR1 0x1u    /* Analog input AN0 */
#define TEMPERATURE_SENSOR2 0x5u    /* Analog input AN1 */
#define TEMPERATURE_SENSOR3 0x9u    /* Analog input AN2 */
#define PRESSURE_SENSOR     0x11u   /* Analog input AN4 */

#define RXSM_LO       PORTBbits.RB3  // Lift-Off
#define RXSM_SODS     PORTBbits.RB1  // Start Of Data Storage
#define RXSM_SOE      PORTBbits.RB2  // Start Of Experiment

#define LO_LED   LATCbits.LATC0  // Lift-Off LED
#define SODS_LED LATCbits.LATC1  // Start Of Data Storage LED
#define SOE_LED  LATCbits.LATC5  // Start Of Experiment LED

#define LASER_CONTROL   LATBbits.LATB4   /* Laser command output */
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

/* 1 unit = 1ms */
#define TIME_LASER_ON    5000   // Timer for Laser Power On after LO (default 5000)
#define TIME_HEATER_OFF  20000  //Timer for Heater Power Off after predefined time (default 15000)
#define RFH_HEATER  5           //Timer for refresh heater power (default X)
#define RFH_ADC     100         //Timer for refresh ADC (default 100)
#define DEBOUNCE_TIME    50     //Timer for debounce system (default 50)
#define TIME_ACQUISITION_OFF    140000  //Timer for stop the camera acquisition (default 140000)


/**
 * @brief Initializes the microcontroller according to the experiment setup.
 * @details Initializes peripherals and I/Os according to the experiment design:
 *          oscillator frequency, required timer period or baudrates...
 */
void board_config(void);

#endif  /* DEF_BOARD_CONFIG_H */

