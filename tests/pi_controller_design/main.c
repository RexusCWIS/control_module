/**
  * @file main.c
  * @brief Project aiming to facilitate the design of the heater controller.
  */

#include <stdint.h>

#include <htc.h>

#include "types.h"
#include "libs/crc.h"
#include "config/pic18f2520_config.h"
#include "config/board_config.h"

#define GET_SYSTEM_TIME(x)      di();               \
                                x = system_time;    \
                                ei();

void uart_send_data(uint8_t data[], uint8_t size);

static serial_frame_s tx_data;

/** @brief Serial line circular buffer. */
static uint8_t serial_tx_buf[16*sizeof(serial_frame_s)];
/** @brief Serial TX ISR increment variable. */
static uint16_t serial_tx_buf_head = 0;
static uint16_t serial_tx_buf_tail = 0;
static uint8_t  serial_tx_is_idle  = 1;

static uint32_t system_time = 0;

static uint8_t adc_conv_timer = 0;
static uint8_t adc_conv_flag  = 0;

void main(void) {

    uint16_t crc16 = 0;

    tx_data.sync[0] = 'U';
    tx_data.sync[1] = 'U';
    tx_data.status = STATUS_POWER_ON;

    board_config();

    /* SODS signal interrupt configuration */
    INTCON2bits.INTEDG1 = 1;
    INTCON3bits.INT1IF  = 0;
    INTCON3bits.INT1IE  = 1;

    /* Start heating */
    CCPR1L = 40u;

    while(1) {
        
        if (adc_conv_flag) {

            /* Reset the flag */
            adc_conv_flag = 0;
            
            /* Get system time */
            GET_SYSTEM_TIME(tx_data.time);

            /* Measure cell temperature */
            ADCON0 = TEMPERATURE_SENSOR1;
            ADC_CONVERSION = 1;
            while (ADC_CONVERSION)
                ;
            tx_data.temperature = (((uint16_t) ADRESH) << 8) + (uint16_t) ADRESL;

            /* Get PWM output */
            tx_data.voltage = CCPR1L;

            /* CRC computation */
            crc16 = crc((uint8_t *) &tx_data, (sizeof(serial_frame_s) - 2u));
            tx_data.crc[0] = (uint8_t) (crc16 >> 8u);
            tx_data.crc[1] = (uint8_t) (crc16 & 0xFFu);

            /* Frame transmission */
            uart_send_data((uint8_t *) &tx_data, sizeof(serial_frame_s));

            if(adc_conv_flag) {
                tx_data.status |= STATUS_OUT_OF_SYNC;
            }
        }
    }
}

void interrupt isr(void) {

    /* Timer interrupt */
    if(TMR0IF) {
        TMR0IF = 0; /* Timer interrupt flag reset */

        /* Reset TMR0 internal counter */
        TMR0H = T0_RELOAD_HIGH;
        TMR0L = T0_RELOAD_LOW;

        system_time++; /* Global time */
        adc_conv_timer++;

        if(adc_conv_timer == 100) {

            adc_conv_flag  = 1;
            adc_conv_timer = 0;
        }
    }

    if(INTCON3bits.INT1F) {

        CCPR1L = 50u;
        INTCON3bits.INT1F = 0;
    }

    /* UART transmission interrupt */
    if (PIR1bits.TXIF) {

        if (serial_tx_buf_head != serial_tx_buf_tail) {

            TXREG = serial_tx_buf[serial_tx_buf_head];

            serial_tx_buf_head++;
            if (serial_tx_buf_head == sizeof(serial_tx_buf)) {
                serial_tx_buf_head = 0u;
            }
        }

        else {
            serial_tx_is_idle = 1;
            PIE1bits.TXIE = 0;
        }

        PIR1bits.TXIF = 0;
    }
}

/** @todo Implement overflow control */
void uart_send_data(uint8_t data[], uint8_t size) {

    uint16_t index = 0u;

    while(index < size) {
        serial_tx_buf[serial_tx_buf_tail] = data[index];
        
        index++;
        
        serial_tx_buf_tail++;
        if(serial_tx_buf_tail == sizeof(serial_tx_buf)) {
            serial_tx_buf_tail = 0u;
        }
    }

    /* If the module was idle, start data transfer. */
    if(serial_tx_is_idle == 1) {
        TXREG = serial_tx_buf[serial_tx_buf_head];
        
        serial_tx_buf_head++;
        if(serial_tx_buf_head == sizeof(serial_tx_buf)) {
            serial_tx_buf_head = 0; 
        }
        serial_tx_is_idle = 0;
    }
    
    PIE1bits.TXIE = 1;
}
