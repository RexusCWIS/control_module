/*-------- REXUS Project -------*\
|*-----CWIS Microcontroller-----*|
|*----------PIC18F2520----------*|
\*-Fabrizio Mancino - Gen. 2013-*/


#include<htc.h>
#define _LEGACY_HEADERS 
#define XTAL_FREQ 20MHZ

#include "pic18f2520 configbits.h"
#include "settings.h"
#include "i2c_slave.h"
#include "i2c_frames.h"



static unsigned char dummy = 0;
static unsigned int i2c_index  = 0;
static unsigned char i2c_dev_reg = 0;

static unsigned char i2c_rx_frame[I2C_RX_FRAME_SIZE];

/* Control loop variables */
int heater_power = 0; 

/* I2C variables */

/** @brief Order sent to the camera module.  */
static i2c_order_e camera_order = STOP_ACQUISITION;
/** @brief I2C frame, holding the data acquired by sensors. */
static i2c_frame_s acquisition_data;

static unsigned char* i2c_rx_registers[1] = {i2c_rx_frame};
static unsigned char* i2c_tx_registers[2] = {(unsigned char *) &camera_order, (unsigned char *) &acquisition_data};
static unsigned char i2c_tx_reg_sizes[2]  = {sizeof(i2c_order_e), sizeof(i2c_frame_s)};

static i2c_state_machine_e i2c_state;

/**
 * @brief Initializes timer parameters.
 * @details Initializes the TMR0 module to get a 1 millisecond tick. 
 */
static void timer_init(void);

unsigned int LOstate=0, SODSstate=0, SOEstate=0, ABstate=0, ABflag=0, conv=0;
unsigned int TimerLaser=0, TimerHeater=0, TimerConv=0, TimerAB=0, TimerAcquisition=0;

void main(void)
{
	

	unsigned int LOenable=0, SODSenable=0, SOEenable=0;
	unsigned int t_cell=0, t_heat=0;
	unsigned long time=0;
	
	settings();	/* Register Init */
	i2c_slave_init(I2C_ADDRESS); /* I2C Init */
	
	/** Check in settings.h , already implemented */
	//timer_init(); /* TMR0 Init */

	while(1)
	{
        /* Sensor data acquisition loop */
		if (conv)
		{
            conv=0; /* reset conversion flag */
			
			/* Measure cell temperature */
			ADCON0 = SENSOR0;
            GO = 1; 
			while(GO) {
			    acquisition_data.temperatures[0].data = (((unsigned int) ADRESH) << 8) + (unsigned int) ADRESL;
			}
			
			/* ONLY FOR TESTS - Send cell temperature through COM port */
				sendtemp(acquisition_data.temperatures[0].data);
		
            /* Measure heater temperature */
			ADCON0=SENSOR1;
			GO = 1;
			while(GO) {
			    acquisition_data.temperatures[1].data = (((unsigned int) ADRESH) << 8) + (unsigned int) ADRESL;
			}
			
			/* ONLY FOR TESTS - Send cell temperature through COM port */
				sendtemp(acquisition_data.temperatures[1].data);
		}
	
		/* LO signal */
		if((LO)&&(!LOenable))
		{
			ABstate=1;
			if((LO)&&(!LOenable)&&(ABflag)) 
			{
				LOstate=1;
				LOenable=1;
				LO_LED=1;
				ABflag=0;
			}
		}
		if(LOstate)
		{
		}
		
		/* SODS signal */
		if((SODS)&&(LOstate)&&(!SODSenable))
		{
			ABstate=1;
			if((SODS)&&(LOstate)&&(!SODSenable)&&(ABflag)) 
			{
				SODSstate=1;
				SODSenable=1;
				SODS_LED=1;
				ABflag=0;				
			}
		}
		if(SODSstate)
		{
			camera_order=START_ACQUISITION;
		}
		
		/* SOE signal */
		if((SOE)&&(SODSstate)&&(LOstate)&&(!SOEenable))
		{
			ABstate=1;
			if((SOE)&&(SODSstate)&&(LOstate)&&(!SOEenable)&&(ABflag)) 
			{
				SOEstate=1;
				SOEenable=1;
				SOE_LED=1;
				ABflag=0;
			}
		}
		if(SOEstate)
		{
			HEATER=255;
		}
	}	
}

/* Send temp value through COM port */
void sendtemp(int temp)
{
	unsigned char tempH, tempL;
	
	tempH=(temp>>8);
	while(!TXIF)
	continue;
	TXREG=tempH;
	
	tempL=temp;
	while(!TXIF)
	continue;
	TXREG=tempL;
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

/* Echo char on COM port */
unsigned char getche(void)
{
	unsigned char c;
	putch(c = getch());
	return c;
}

/* ISR */
void interrupt isr(void)
{
	if (TMR0IF) /* TMR0 overflow interrupt */
	{
		TMR0IF = 0; /* Timer interrupt flag reset */
		
		/* Reset TMR0 internal counter */
		TMR0H = T0_RELOAD_HIGH;
        TMR0L = T0_RELOAD_LOW;
		
		acquisition_data.time++; /* Global time */
		
		/* Timer for Laser Power On after LO */
		if(LOstate)
		{
			TimerLaser++;
		}
		
		if (TimerLaser>=TEMPOLASER)
		{
			LASER=1; /* Laser power on */
		}
		
		/* Timer for stop the camera acquisition */
		if(SODSstate)
		{
			TimerAcquisition++;
		}
		if(TimerAcquisition>=TEMPOACQUISITION)
		{
			camera_order = STOP_ACQUISITION;
			TimerAcquisition=0;
		}
		
		/* Timer for Heater Power On after SOE */
		if(SOEstate)
		{
			//TimerHeater++; /* Off for tests */
		}
	
        /* Heater control loop */
        /** @todo No need to keep the TimerHeater variable, directly use the system time variable */
		if (TimerHeater <= TEMPOHEATER) {
            /* Compute the error between the setpoint and the actual temperature */
            heater_power = (int) acquisition_data.temperatures[0].data - TEMPERATURE_CONTROL_SETPOINT;
            /* Multiply it by the proportional gain */
            heater_power *= TEMPERATURE_CONTROL_PGAIN;

            /* Apply system limits (no cooling, 8-bit duty cycle) */
            heater_power = (heater_power < 0) ? 0 : heater_power;
            HEATER = (unsigned char) (heater_power && 0xFFu); 
		}

		/* Timer for Heater Power Off after predefined time */
        else {
            HEATER = 0;     /* Power-off heater */ 
            SOEstate = 0; 
        }
		
		/* Timer for each conversion */
		if (TimerConv>=TEMPOCONV)
		{
			conv=1; /* Starts adc conversion */
			TimerConv=0;
		}
		TimerConv++;
		
		/* Timer for debounce system */
		if(ABstate)
		{
			TimerAB++;
			ABstate=0;
		}
		else
		{
			TimerAB=0;
		}
		
		if(TimerAB>=TEMPOAB)
		{
			ABflag=1;
			TimerAB=0;
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

static void timer_init(void) {

    /* Set the internal oscillator to 8MHz */
    OSCCON |= 0x70;  

    /* Use timer 0 to generate an interrupt each second */
    T0CONbits.TMR0ON = 0;   /* Disable timer */
    T0CON &= ~0xF8;         /* Set the timer in 16-bit mode */
    T0CON |= 0x7;           /* Set a 1:256 prescaler */

    TMR0H = T0_RELOAD_HIGH;
    TMR0L = T0_RELOAD_LOW;

    T0CONbits.TMR0ON = 1;   /* Restart timer */
}
