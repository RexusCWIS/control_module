/**
 * @file main_CV.c
 * @brief CWIS data control module.
 */

#include <stdint.h>

#include <htc.h>
#define _LEGACY_HEADERS
#define XTAL_FREQ 20MHZ

#include "drivers/types.h"

#include "config/pic18f2520_config.h"
#include "config/board_config.h"

#include "drivers/eeprom.h"
#include "drivers/i2c_slave.h"

#include "libs/controller.h"
#include "libs/crc.h"

int16_t abs16(int16_t a, int16_t b);
uint8_t sensor_value_in_range(uint16_t value);

void load_eeprom_data(void);

void lo_signal_handler(void);
void sods_signal_handler(void);
void soe_signal_handler(void);
void power_off_handler(void);

void uplink_heater_handler(void);
void uplink_rxsm_handler(void);
void uplink_laser_handler(void);
void uplink_camera_handler(void);
void uplink_config_handler(void);

/** @brief Global system time. */
static uint32_t system_time;

static module_control_mode_e control_mode = CONTROL_AUTOMATIC;
static experiment_state_e current_experiment_state = BOOT;

/** @brief Status flags stored in EEPROM at the end of each run. */
static uint8_t eeprom_flags = 0;

/** @brief Dummy 8-bit variable, used to empty serial buffers. */
static uint8_t dummy = 0;

/* UART variables */

/** @brief Increments an index of a circular buffer with overflow control. */
#define CIRCULAR_BUF_INDEX_INCR(index, size)    do {                        \
                                                    index++;                \
                                                    if(index == size) {     \
                                                        index = 0;          \
                                                    }                       \
                                                } while(0)

/** @brief Returns the distance between the tail and head of a circular buffer. */
#define CIRCULAR_BUF_INDEX_DISTANCE(head, tail, size)   ((tail >= head) ?   \
                                                        (tail - head) :     \
                                                        (tail + size - head))



/** @brief Downlink data frame. */
static serial_frame_s dl_data = {{'U', 'U'}, {0, {0, 0, 0}, 0, 0, 0},
                                {0, 0, 0}, {0, 0}, {0, 0}};
/** @brief Serial line TX circular buffer. */
static uint8_t serial_tx_buf[16*sizeof(serial_frame_s)];
/** @brief Serial TX ISR increment variable. */
static uint16_t serial_tx_buf_head = 0;
static uint16_t serial_tx_buf_tail = 0;
static uint8_t  serial_tx_is_idle  = 1;

/** @brief Serial line RX circular buffer size. */
#define SERIAL_RX_BUF_SIZE  128
/** @brief Serial line RX circular buffer. */
static uint8_t serial_rx_buf[SERIAL_RX_BUF_SIZE];

/** @brief Serial RX ISR increment variables. */
static uint8_t serial_rx_buf_head = 0;
static uint8_t serial_rx_buf_tail = 0;
static uplink_command_e current_uplink_command = UPLINK_NONE;
static uint8_t uplink_options[UPLINK_COMMAND_SIZE];

/* I2C variables */

/** @brief I2C ISR index increment variable. */
static uint16_t i2c_index  = 0;
/** @brief Holds the current device register accessed by the I2C bus master. */
static uint8_t i2c_dev_reg = 0;


/** @brief Order sent to the camera module.  */
static i2c_order_e camera_order = STOP_ACQUISITION;

/** @brief Array of I2C device registers writeable by the I2C bus master. */
static uint8_t* i2c_rx_registers[1] = {(uint8_t *) &dl_data.camera_data};
/** @brief Array of I2C device registers readable by the I2C bus master. */
static uint8_t* i2c_tx_registers[2] = {(uint8_t *) &camera_order, (uint8_t *) &dl_data.acquired_data};
/** @brief Size of the writable device registers arrays. */
static uint8_t i2c_rx_reg_sizes[1]  = {sizeof(i2c_camera_data_s)};
/** @brief Size of the readable device registers arrays. */
static uint8_t i2c_tx_reg_sizes[2]  = {sizeof(i2c_order_e), sizeof(i2c_frame_s)};

/** @brief I2C ISR state machine. */
static i2c_state_machine_e i2c_state;

/** @brief Message displayed at boot. */
static unsigned char boot_msg[68]  = "\n\rThis is the CWIS control module. Starting functional tests...\n\r\n\r\0";

/* Control loop variables */

static int heater_power = 0;
static uint32_t control_pgain = TEMPERATURE_CONTROL_PGAIN;
static uint32_t control_dti = TEMPERATURE_CONTROL_DTI;
static uint16_t *control_temperature = &(dl_data.acquired_data.temperatures[0]);
static uint32_t control_setpoint = 0;


/**
 * @brief Initializes timer parameters.
 * @details Initializes the TMR0 module to get a 1 millisecond tick. 
 */
static void timer_init(void);

static uint32_t power_timer, debounce_timer = 0,
                acquisition_timer = 0;

static uint8_t  debounce_flags = 0, adc_conv_flag = 0, adc_conv_timer = 0;
static uint32_t lo_debounce_time = 0, sods_debounce_time = 0,
                soe_debounce_time = 0;

void uart_send_data(uint8_t data[], uint8_t size);

void main(void) {

    uint16_t crc16 = 0;

    dl_data.sync[0] = 'U';
    dl_data.sync[1] = 'U';
    dl_data.acquired_data.status |= STATUS_POWER_ON;

    board_config(); /* Register Init */
    i2c_slave_init(I2C_ADDRESS); /* I2C Init */

    load_eeprom_data();

    pi_controller_s pi_controller;
    init_pi_controller(&pi_controller, control_pgain, control_dti, 
                       TEMPERATURE_CONTROL_LOWER_SAT,
                       TEMPERATURE_CONTROL_UPPER_SAT);

    uart_send_data(boot_msg, sizeof(boot_msg)); 


    while (1) {

        /* Sensor data acquisition loop */
        if (adc_conv_flag) {
            
            adc_conv_flag = 0; /* reset conversion flag */

            /* Get system time */
            di();
            dl_data.acquired_data.time = system_time;
            ei();

            /* Measure cell temperature */
            ADCON0 = TEMPERATURE_SENSOR1;
            ADC_CONVERSION = 1;
            while (ADC_CONVERSION)
                ;
            di();
            dl_data.acquired_data.temperatures[0] = (((uint16_t) ADRESH) << 8) + (uint16_t) ADRESL;
            ei();

            /* Measure heater temperature */
            ADCON0 = TEMPERATURE_SENSOR2;
            ADC_CONVERSION = 1;
            while (ADC_CONVERSION)
                ;
            di();
            dl_data.acquired_data.temperatures[1] = (((uint16_t) ADRESH) << 8) + (uint16_t) ADRESL;
            ei();

            /* Measure ambient temperature */
            ADCON0 = TEMPERATURE_SENSOR3;
            ADC_CONVERSION = 1;
            while (ADC_CONVERSION)
                ;
            di();
            dl_data.acquired_data.temperatures[2] = (((uint16_t) ADRESH) << 8) + (uint16_t) ADRESL;
            ei();

            /* Measure ambient pressure */
            ADCON0 = PRESSURE_SENSOR;
            ADC_CONVERSION = 1;
            while (ADC_CONVERSION)
                ;
            di();
            dl_data.acquired_data.pressure = (((uint16_t) ADRESH) << 8) + (uint16_t) ADRESL;
            ei();

            dl_data.acquired_data.heating = HEATER;

            /* CRC computation */
            crc16 = crc((uint8_t *) &dl_data, (sizeof(serial_frame_s) - 2u));
            dl_data.crc[0] = (uint8_t) (crc16 >> 8u);
            dl_data.crc[1] = (uint8_t) (crc16 & 0xFFu);

            /* Frame transmission */
            uart_send_data(&dl_data, sizeof(serial_frame_s));

            /* Heater control loop */
            if(control_mode == CONTROL_AUTOMATIC) {
                if (dl_data.acquired_data.status & STATUS_HEATER_ON) {

                    /* Compute the error between the setpoint and the actual temperature */
                    int32_t error = control_setpoint - (int32_t) *control_temperature;
                    
                    HEATER = (uint8_t) (pi_control_loop(&pi_controller, error) >> 14);
                }

                else {
                    /* Switch off the heater */
                    HEATER = 0;
                }
            }
        }

        /* LO signal debounce */
        if((RXSM_LO && !(debounce_flags & LO_DEBOUNCED_STATE)) ||
           !(RXSM_LO) && (debounce_flags & LO_DEBOUNCED_STATE)) {

            if(!(debounce_flags & LO_DEBOUNCE_REQUEST)) {

                lo_debounce_time = system_time;
                debounce_flags  |= LO_DEBOUNCE_REQUEST;
            }

            if((system_time - lo_debounce_time) >= DEBOUNCE_TIME) {
                debounce_flags ^= LO_DEBOUNCED_STATE;
                debounce_flags &= ~LO_DEBOUNCE_REQUEST;
            }
        }

        else if((debounce_flags & LO_DEBOUNCE_REQUEST)) {
            debounce_flags &= ~LO_DEBOUNCE_REQUEST;
        }


        /* SODS signal debounce */
        if((RXSM_SODS && !(debounce_flags & SODS_DEBOUNCED_STATE)) ||
           !(RXSM_SODS) && (debounce_flags & SODS_DEBOUNCED_STATE)) {

            if(!debounce_flags & SODS_DEBOUNCE_REQUEST) {

                sods_debounce_time = system_time;
                debounce_flags  |= SODS_DEBOUNCE_REQUEST;
            }

            if((system_time - sods_debounce_time) >= DEBOUNCE_TIME) {
                debounce_flags ^= SODS_DEBOUNCED_STATE;
                debounce_flags &= ~SODS_DEBOUNCE_REQUEST;
            }
        }

        else if(debounce_flags & SODS_DEBOUNCE_REQUEST) {
            debounce_flags &= ~SODS_DEBOUNCE_REQUEST;
        }

        /* SOE signal debounce */
        if((RXSM_SOE && !(debounce_flags & SOE_DEBOUNCED_STATE)) ||
           !(RXSM_SOE) && (debounce_flags & SOE_DEBOUNCED_STATE)) {

            if(!debounce_flags & SOE_DEBOUNCE_REQUEST) {

                soe_debounce_time = system_time;
                debounce_flags  |= SOE_DEBOUNCE_REQUEST;
            }

            if((system_time - soe_debounce_time) >= DEBOUNCE_TIME) {
                debounce_flags ^= SOE_DEBOUNCED_STATE;
                debounce_flags &= ~SOE_DEBOUNCE_REQUEST;
            }
        }

        else if(debounce_flags & SOE_DEBOUNCE_REQUEST) {
            debounce_flags &= ~SOE_DEBOUNCE_REQUEST;
        }


        /* Actions taken on RXSM signals */

        /* LO signal */
        if ((debounce_flags & LO_DEBOUNCED_STATE) && (current_experiment_state == BOOT)) {
            lo_signal_handler();
        }

        /* SODS signal */
        if ((debounce_flags & SODS_DEBOUNCED_STATE) && (current_experiment_state == LO)) {
            sods_signal_handler();
        }

        /* SOE signal */
        if ((debounce_flags & SOE_DEBOUNCED_STATE) && (current_experiment_state == SODS)) {
            soe_signal_handler();
        }

        /* Handle uplink commands */
        switch(current_uplink_command) {
            case UPLINK_HEATER:
                uplink_heater_handler();
                current_uplink_command = UPLINK_NONE;
                break;

            case UPLINK_RXSM:
                uplink_rxsm_handler();
                current_uplink_command = UPLINK_NONE;
                break;

            case UPLINK_LASER:
                uplink_laser_handler();
                current_uplink_command = UPLINK_NONE;
                break;

            case UPLINK_CAMERA:
                uplink_camera_handler();
                current_uplink_command = UPLINK_NONE;
                break;

            case UPLINK_CONFIG:
                uplink_config_handler();
                current_uplink_command = UPLINK_NONE;
                break;

            /* Uplink command and options definition */
            case UPLINK_NONE:
                if(CIRCULAR_BUF_INDEX_DISTANCE(serial_rx_buf_head,
                                               serial_rx_buf_tail,
                                               SERIAL_RX_BUF_SIZE)
                        > UPLINK_COMMAND_SIZE) {

                    current_uplink_command = serial_rx_buf[serial_rx_buf_head];
                    CIRCULAR_BUF_INDEX_INCR(serial_rx_buf_head, SERIAL_RX_BUF_SIZE);

                    /* Get all uplink command options */
                    uint8_t index;
                    for(index = 0; index < UPLINK_COMMAND_SIZE; index++) {
                        uplink_options[index] = serial_rx_buf[serial_rx_buf_head];
                        CIRCULAR_BUF_INDEX_INCR(serial_rx_buf_head,
                                                SERIAL_RX_BUF_SIZE);
                    }
                }

                break;

            default:
                current_uplink_command = UPLINK_NONE;
        }
    }
}

/* ISR */
void interrupt isr(void) 
{
    /* TMR0 overflow interrupt */
    if (TMR0IF) {

        TMR0IF = 0; /* Timer interrupt flag reset */

        /* Reset TMR0 internal counter */
        TMR0H = T0_RELOAD_HIGH;
        TMR0L = T0_RELOAD_LOW;

        system_time++; /* Global time */
        adc_conv_timer++;

        /* Timer for stop the camera acquisition */
        if (system_time == acquisition_timer) {
            camera_order  = STOP_ACQUISITION;
            eeprom_flags |= EEPROM_FLAGS_EODS;
            eeprom_write_byte(eeprom_flags, EEPROM_FLAGS_ADDR);
        }

        /* Timer for heater off */
        if (system_time == power_timer) {
            power_off_handler();
        }

        /* ADC conversion rate */
        /** @todo Replace the modulo operation for performance */
        if (adc_conv_timer == RFH_ADC) {
            adc_conv_flag  = 1; /* Starts adc conversion */
            adc_conv_timer = 0;
        }
    }

    /* I2C interrupt */
    if (SSPIF) {
        /* Master READ */
        if (SSPSTATbits.R_nW) {

            /* Last byte was a memory address */
            if (!SSPSTATbits.D_nA) {

                SSPBUF = i2c_tx_registers[i2c_dev_reg][0u];
                i2c_index = 1u; /* Clear index */
                SSPCON1bits.CKP = 1; /* Release I2C clock */
            }

            /* Last byte was data (the slave is transmitting the frame) */
            if (SSPSTATbits.D_nA) {

                if (i2c_index < i2c_tx_reg_sizes[i2c_dev_reg]) {
                    SSPBUF = i2c_tx_registers[i2c_dev_reg][i2c_index];
                    i2c_index++;
                }                    /* If we're done transmitting the frame, empty the I2C buffer */
                else {
                    dummy = SSPBUF;
                }

                SSPCON1bits.CKP = 1; /* Release I2C clock */
            }
        }

        /* Master WRITE */
        if (!SSPSTATbits.R_nW) {

            /* Last byte was a memory address */
            if (!SSPSTATbits.D_nA) {
                dummy = SSPBUF; /* Clear I2C buffer */
                i2c_state = I2C_SET_DEV_REG;
                SSPCON1bits.CKP = 1; /* Release I2C clock */
            }

            if (SSPSTATbits.D_nA) {

                if (i2c_state == I2C_SET_DEV_REG) {
                    i2c_dev_reg = SSPBUF;
                    i2c_index = 0;
                    i2c_state = I2C_DATA;
                }
                else {
                    if (i2c_index < I2C_RX_FRAME_SIZE) {
                        i2c_rx_registers[i2c_dev_reg][i2c_index] = SSPBUF;
                        i2c_index++;
                    }
                    else {
                        dummy = SSPBUF; /* Clear I2C buffer */
                    }
                }

                /* Write collision handling */
                if (SSPCON1bits.WCOL) {
                    SSPCON1bits.WCOL = 0; /* Clear collision flag */
                    dummy = SSPBUF; /* Clear I2C buffer */
                }

                SSPCON1bits.CKP = 1; /* Release I2C clock */
            }
        }

        SSPIF = 0;
    }

    /* I2C Bus collision interrupt */
    if(BCLIF) {

        dummy = SSPBUF;         /* Clear I2C buffer */
        BCLIF = 0;              /* Clear bus collision flag */
        SSPCON1bits.CKP = 1;    /* Release I2C clock */
        SSPIF = 0;              /* Clear I2C interrupt flag */
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

    if(PIR1bits.RCIF) {

        /* Record received byte */
        serial_rx_buf[serial_rx_buf_tail] = RCREG;

        /* Increment buffer tail index and check for overflow */
        CIRCULAR_BUF_INDEX_INCR(serial_rx_buf_tail, SERIAL_RX_BUF_SIZE);
        
        PIR1bits.RCIF = 0;
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

void uplink_heater_handler(void) {

    control_mode = CONTROL_MANUAL;
    HEATER = uplink_options[0];
}

void uplink_rxsm_handler(void) {

    switch(uplink_options[0]) {
        case 0x01u:
            lo_signal_handler();
        break;

        case 0x3u:
            sods_signal_handler();
            break;

        case 0x5u:
            soe_signal_handler();
            break;

        default:
            break;
    }
}

void uplink_laser_handler(void) {
    
    if(uplink_options[0] == 0xFF) {
        LASER_CONTROL = 1;
    }

    else {
        LASER_CONTROL = 0;
    }
}

void uplink_camera_handler(void) {

    if(uplink_options[0]) {
        camera_order = START_ACQUISITION;
    }

    else {
        camera_order = STOP_ACQUISITION;
    }
}

void uplink_config_handler(void) {

    switch(uplink_options[0]) {
        
        /* PI control parameters */
        case 0x00u:
            eeprom_write_byte(uplink_options[1], EEPROM_PGAIN_ADDR);
            eeprom_write_byte(uplink_options[2], EEPROM_DTI_ADDR);
            break;
            
        default:
            break;
    }
}

int16_t abs16(int16_t a, int16_t b) {

    int16_t c = a - b;

    if(c < 0) {
        c = -c;
    }

    return c;
}

uint8_t sensor_value_in_range(uint16_t value) {

    return((value < TEMPERATURE_RANGE_HIGH) &&
           (value > TEMPERATURE_RANGE_LOW));
}

void load_eeprom_data(void) {

    /*
    uint32_t rvalue = 0;

    eeprom_read_array((uint8_t *) &rvalue, sizeof(uint32_t), EEPROM_PGAIN_ADDR);

    if(rvalue != 0) {
        control_pgain = rvalue;
    }

    eeprom_read_array((uint8_t *) &rvalue, sizeof(uint32_t), EEPROM_DTI_ADDR);
    if(rvalue != 0) {
        control_dti = rvalue;
    }
    */

    eeprom_flags = eeprom_read_byte(EEPROM_FLAGS_ADDR);
}

void lo_signal_handler(void) {

    /* LO commands */
    power_timer = system_time + TIME_POWER_OFF; /* Set time for power-off */

    /* Switch laser on */
    current_experiment_state = LO;
    LASER_CONTROL = 1;
    dl_data.acquired_data.status |= STATUS_LO | STATUS_LASER_ON;
    LO_LED = 1;
}

void sods_signal_handler(void) {

    /* SODS commands */
    camera_order = START_ACQUISITION;                       /* Camera start acquisition */
    acquisition_timer = system_time + TIME_ACQUISITION_OFF; /* Set time for stop acquisition */

    current_experiment_state = SODS;
    dl_data.acquired_data.status |= STATUS_SODS;
    LO_LED = 0;
    SODS_LED = 1;
}

void soe_signal_handler(void) {

    /* Check for sensor defects and compute the temperature setpoint */

    /* Verify that probes 1 and 2 do not differ from more than 4 degrees */
    int16_t diff = abs16(dl_data.acquired_data.temperatures[0],
                         dl_data.acquired_data.temperatures[1]);

    int16_t diff1, diff2;

    /* If they do, compare their value to the base of the cell */
    if(diff > (TEMPERATURE_ONE_DEGREE << 2)) {
        diff1 = abs16(dl_data.acquired_data.temperatures[0],
                      dl_data.acquired_data.temperatures[2]);
        diff2 = abs16(dl_data.acquired_data.temperatures[1],
                      dl_data.acquired_data.temperatures[2]);

        /* Use the value closest to the base of the cell as the control sensor */
        if((diff1 > diff2) &&
           sensor_value_in_range(dl_data.acquired_data.temperatures[0])) {
            control_temperature = &(dl_data.acquired_data.temperatures[1]);
        }
    }

    control_setpoint = dl_data.acquired_data.temperatures[0] +
                       TEMPERATURE_CONTROL_STEP;

    current_experiment_state = SOE;
    dl_data.acquired_data.status |= (STATUS_SOE | STATUS_HEATER_ON);
    SODS_LED = 0;
    SOE_LED = 1;
}

void power_off_handler(void) {

    /* Disable laser and heater (the heater is disabled in the main loop) */
    LASER_CONTROL = 0;
    dl_data.acquired_data.status &= ~(STATUS_HEATER_ON | STATUS_LASER_ON);
}
