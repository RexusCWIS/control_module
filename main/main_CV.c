/*-------- REXUS Project -------*\
|*-----CWIS Microcontroller-----*|
|*----------PIC18F2520----------*|
\*-Fabrizio Mancino - Gen. 2013-*/


#include<htc.h>
#define _LEGACY_HEADERS 
#define XTAL_FREQ 20MHZ

#include "config/pic18f2520_config.h"
#include "config/board_config.h"
#include "drivers/i2c_slave.h"
#include "i2c_frames.h"



static unsigned char dummy = 0;

/* Control loop variables */
int heater_power = 0; 

/* I2C variables */

/** @brief I2C ISR index increment variable. */
static unsigned int i2c_index  = 0;
/** @brief Holds the current device register accessed by the I2C bus master. */
static unsigned char i2c_dev_reg = 0;
/** @brief Array holding the data received on the I2C bus. */
static unsigned char i2c_rx_frame[I2C_RX_FRAME_SIZE];

/** @brief Order sent to the camera module.  */
static i2c_order_e camera_order = STOP_ACQUISITION;
/** @brief I2C frame, holding the data acquired by sensors. */
static i2c_frame_s acquisition_data;

/** @brief Array of I2C device registers writeable by the I2C bus master. */
static unsigned char* i2c_rx_registers[1] = {i2c_rx_frame};
/** @brief Array of I2C device registers readable by the I2C bus master. */
static unsigned char* i2c_tx_registers[2] = {(unsigned char *) &camera_order, (unsigned char *) &acquisition_data};
/** @brief Size of the readable device registers arrays. */
static unsigned char i2c_tx_reg_sizes[2]  = {sizeof(i2c_order_e), sizeof(i2c_frame_s)};

/** @brief I2C ISR state machine. */
static i2c_state_machine_e i2c_state;

/**
 * @brief Initializes timer parameters.
 * @details Initializes the TMR0 module to get a 1 millisecond tick. 
 */
static void timer_init(void);

unsigned int LOstate = 0, SODSstate = 0, SOEstate = 0, DEBOUNCEstate = 0, DEBOUNCEflag = 0, conv = 0;
unsigned int TimerLaser = 0, TimerHeater = 0, TimerDebounce = 0;
unsigned long int TimerAcquisition = 0;

void main(void)
{
	unsigned int LOenable = 0, SODSenable = 0, SOEenable = 0;
	unsigned int t_cell = 0, t_heat = 0;
	
	board_config();              /* Register Init */
	i2c_slave_init(I2C_ADDRESS); /* I2C Init */
	

	while(1)
	{
        /* Sensor data acquisition loop */
		if (conv)
		{
            conv = 0;   /* reset conversion flag */
			
			/* Measure cell temperature */
			ADCON0 = SENSOR0;
            GO = 1; 
			while(GO)
			acquisition_data.temperatures[0].data = (((unsigned int) ADRESH) << 8) + (unsigned int) ADRESL;
			
			/* ONLY FOR TESTS - Send cell temperature through COM port */
			sendtemp(acquisition_data.temperatures[0].data);
		
            /* Measure heater temperature */
			ADCON0 = SENSOR1;
			GO = 1;
			while(GO)
			acquisition_data.temperatures[1].data = (((unsigned int) ADRESH) << 8) + (unsigned int) ADRESL;

			/* ONLY FOR TESTS - Send cell temperature through COM port */
			sendtemp(acquisition_data.temperatures[1].data);
		}
	
		/* LO signal */
		if((LO)&&(!LOenable))
		{
			DEBOUNCEstate=1;
			if((LO)&&(!LOenable)&&(DEBOUNCEflag)) 
			{
				LOstate=1;
				LOenable=1;
				LO_LED=1;
				DEBOUNCEflag=0;
				
				/* LO commands */
				TimerLaser=acquisition_data.time+ON_LASER; /* Set time for laser on */
			}
		}
		
		/* SODS signal */
		if((SODS)&&(LOenable)&&(!SODSenable))
		{
			DEBOUNCEstate=1;
			if((SODS)&&(LOenable)&&(!SODSenable)&&(DEBOUNCEflag)) 
			{
				SODSstate=1;
				SODSenable=1;
				SODS_LED=1;
				DEBOUNCEflag=0;
				
				LOstate=0;
				LO_LED=0;
				
				
				/* SODS commands */
				camera_order=START_ACQUISITION; /* Camera start acquisition */
				TimerAcquisition=acquisition_data.time+OFF_ACQUISITION; /* Set time for stop acquisition */
			}
		}
		
		/* SOE signal */
		if((SOE)&&(SODSenable)&&(LOenable)&&(!SOEenable))
		{
			DEBOUNCEstate=1;
			if((SOE)&&(SODSenable)&&(LOenable)&&(!SOEenable)&&(DEBOUNCEflag)) 
			{
				SOEstate=1;
				SOEenable=1;
				SOE_LED=1;
				DEBOUNCEflag=0;
				
				SODSstate=0;
				SODS_LED=0;
				
				/* SOE commands */
				TimerHeater=acquisition_data.time+OFF_HEATER; /* Set time for heater off */
				
			}
			
			if(SOEstate)
			{
				/* Heater control loop */
				if (acquisition_data.time%RFH_HEATER==0) 
				{
					/* Compute the error between the setpoint and the actual temperature */
					heater_power = TEMPERATURE_CONTROL_SETPOINT -(int) acquisition_data.temperatures[0].data;
					/* Multiply it by the proportional gain */
					heater_power *= TEMPERATURE_CONTROL_PGAIN;

					/* Apply system limits (no cooling, 8-bit duty cycle) */
					heater_power = (heater_power < 0) ? 0 : heater_power; 
					HEATER = (unsigned char) (heater_power && 0xFFu); 
				}
			}
		}
	}	
}

/* Send temp value through COM port */
void sendtemp(int temp) 
{

	unsigned char tempH, tempL;	

    tempH = (temp >> 8);
	while(!TXIF)
	continue;
	TXREG = tempH;
	
	tempL = temp;
	while(!TXIF)
	continue;
	TXREG = tempL;
}

/* Put char on COM port */
void putch(unsigned char byte) 
{

	/* output one byte */
	while(!TXIF)	/* set when register is empty */
		continue;
	TXREG = byte;
}

/* Get char on COM port */
unsigned char getch() 
{
	/* retrieve one byte */
	while(!RCIF)	/* set when register is not empty */
		continue;
	return RCREG;	
}

/* ISR */
void interrupt isr(void) 
{
	if (TMR0IF) 
	{ /* TMR0 overflow interrupt */
		
        TMR0IF = 0; /* Timer interrupt flag reset */
		
		/* Reset TMR0 internal counter */
		TMR0H = T0_RELOAD_HIGH;
        TMR0L = T0_RELOAD_LOW;
		
		acquisition_data.time++; /* Global time */
		
		/* Timer for laser on */
		if (acquisition_data.time == TimerLaser) 
		{
			LASER = 1; /* Laser power on */
		}
		
		/* Timer for stop the camera acquisition */
		if(acquisition_data.time == TimerAcquisition) 
		{
			camera_order = STOP_ACQUISITION;
		}
		
		/* Timer for heater off */
		if(acquisition_data.time == TimerHeater) 
		{
			HEATER = 0;
		}
		
		/* Timer for refresh ADC */
		if ((acquisition_data.time % RFH_ADC) == 0) 
		{
			conv = 1; /* Starts adc conversion */
		}
		
		/* Timer for debounce system */
		if(DEBOUNCEstate) 
		{
			TimerDebounce++;
			DEBOUNCEstate = 0;
		}
        else 
		{
			TimerDebounce = 0;
		}

		if(TimerDebounce >= DEBOUNCE) 
		{
			DEBOUNCEflag = 1;
			TimerDebounce = 0;
		}
	}
	
	/* I2C interrupt */
    if(SSPIF) {
        /* Master READ */
        if(SSPSTATbits.R_nW) {

            /* Last byte was a memory address */
            if(!SSPSTATbits.D_nA) {

                SSPBUF = i2c_tx_registers[i2c_dev_reg][0u];
                i2c_index = 1u;         /* Clear index */
                SSPCON1bits.CKP = 1;    /* Release I2C clock */
            }

            /* Last byte was data (the slave is transmitting the frame) */
            if(SSPSTATbits.D_nA) {

                if (i2c_index < i2c_tx_reg_sizes[i2c_dev_reg]) {
                    SSPBUF = i2c_tx_registers[i2c_dev_reg][i2c_index];
                    i2c_index++;
                }
                    /* If we're done transmitting the frame, empty the I2C buffer */
                else {
                    dummy = SSPBUF;
                }

                SSPCON1bits.CKP = 1;    /* Release I2C clock */
            }
        }

        /* Master WRITE */
        if(!SSPSTATbits.R_nW) {

            /* Last byte was a memory address */
            if(!SSPSTATbits.D_nA) {
                dummy = SSPBUF;         /* Clear I2C buffer */
                i2c_state = I2C_SET_DEV_REG;
                SSPCON1bits.CKP = 1;    /* Release I2C clock */
            }

            if(SSPSTATbits.D_nA) {

                if(i2c_state == I2C_SET_DEV_REG) {
                    i2c_dev_reg = SSPBUF;
                    i2c_index = 0;
                    i2c_state = I2C_DATA; 
                }

                else {
                    if(i2c_index < I2C_RX_FRAME_SIZE) {
                        i2c_rx_registers[i2c_dev_reg][i2c_index] = SSPBUF;
                        i2c_index++;
                    }

                    else {
                        dummy = SSPBUF;     /* Clear I2C buffer */
                    }
                }
                
                /* Write collision handling */
                if(SSPCON1bits.WCOL) {
                    SSPCON1bits.WCOL = 0;   /* Clear collision flag */
                    dummy = SSPBUF;         /* Clear I2C buffer */
                }

                SSPCON1bits.CKP = 1;        /* Release I2C clock */
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
}
