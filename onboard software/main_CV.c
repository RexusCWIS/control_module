/*-------- REXUS Project -------*\
|*-----CWIS Microcontroller-----*|
|*------------main.c------------*|
|*----------PIC18F2520----------*|
\*--Versione 1.0.0 - Gen. 2013--*/ //508 25,56 666 36,62

#include "pic18f2520 configbits.h"
#include<htc.h>
#define _LEGACY_HEADERS 
#define XTAL_FREQ 20MHZ


#include "settings.h"
#include "delay.h"
#include "i2c_slave.h"
#include "i2c_frames.h"



static unsigned char dummy = 0;
static unsigned int i2c_index  = 0;
static unsigned char i2c_dev_reg = 0;

static unsigned char i2c_rx_frame[I2C_RX_FRAME_SIZE];

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

int LOstate=0, SODSstate=0, SOEstate=0, ABstate=0, ABflag=0, conv=0;
int TimerLaser=0, TimerHeater=0, TimerConv=0, TimerAB=0;

void main(void)
{
	

	int LOenable=0, SODSenable=0, SOEenable=0, chconv=0, n_avg_cell=0, n_avg_heat=0;
	unsigned int t_cell=0, t_heat=0, sum_cell=0, sum_heat=0, avg_cell=0, avg_heat=0, tr_heat=0;
	unsigned long time=0;
	settings(); // Imposto le porte come da settings.h	
	
	i2c_slave_init(I2C_ADDRESS); //I2C Init
	timer_init(); //TMR0 Init

	
	while(1)
	{
        /* Sensor data acquisition loop */
		if (conv)
		{
            /* Measure cell temperature */
			ADCON0 = SENSOR0;
            GO = 1; 
			while(GO) {
			    acquisition_data.temperatures[0].data = (((unsigned int) ADRESH) << 8) + (unsigned int) ADRESL;
			}
		
            /* Measure heater temperature */
			ADCON0=SENSOR1;
			GO = 1;
			while(GO) {
			    acquisition_data.temperatures[1].data = (((unsigned int) ADRESH) << 8) + (unsigned int) ADRESL;
			}
			
			/* avg_cell=t_cell;
			   avg_heat=t_heat; */
			
			//SOE_LED=SOE_LED^1;
			//SODS_LED=SODS_LED^1;
			//LO_LED=LO_LED^1;
		}

		
		
		if((LO)&&(!LOenable)) //LO Signal
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
		
		if((SODS)&&(LOstate)&&(!SODSenable)) //SODS Signal
		{
			ABstate=1;
			if((SODS)&&(LOstate)&&(!SODSenable)&&(ABflag)) 
			{
				SODSstate=1;
				SODSenable=1;
				SODS_LED=1;
				ABflag=0;			
				camera_order=START_ACQUISITION;
			}
		}
		
		if((SOE)&&(SODSstate)&&(LOstate)&&(!SOEenable)) //SOE Signal
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
		/*tr_heat++;
		if((SOEstate)&&(tr_heat>=TR_HEATER))
		{
			if((avg_heat-avg_cell)>146)
			{
				HEATER=0;
			}
			else
			{
				if((avg_heat-avg_cell)>(146-(TLINEARE*15)))
				{
					HEATER=(avg_cell+146-avg_heat)*(255/(TLINEARE*15));
				}
				else
				{
					HEATER=255;
				}
			}
			tr_heat=0;
		}
		*/
	}
	
	
	
	
	
	
	
	
	
	
//	DelayMs(250);
//	DelayMs(250);
	/*
	while(1)
	{				
		
		if(conv)
		{
			conv=0;
			//chconv=1; //provvisorio
			switch (chconv)
			{
				case 0:
				chconv=chconv^1;
				ADCON0=SENSOR0;
				//DelayUs(250);
				GO=1;
				while(GO)
				//DelayUs(250);
				t_cell=(ADRESH<<8)+ADRESL;
				
				if(n_avg_cell<DIMAVG)
				{
					sum_cell=+t_cell;
					n_avg_cell++;
				}
				else
				{
					avg_cell=sum_cell/(n_avg_cell-1);
					sum_cell=0;
					n_avg_cell=0;
					
					sendtemp(avg_cell);
				}
				sendtemp(t_cell); //temporaneo test
				break;
				
				case 1:
				chconv=chconv^1;
				ADCON0=SENSOR1;
				DelayUs(250);
				GO=1;
				while(GO)
				DelayUs(250);
				t_heat=(ADRESH<<8)+ADRESL;
				
				if(n_avg_heat<DIMAVG)
				{
					sum_heat=+t_heat;
					n_avg_heat++;
				}
				else
				{
					avg_heat=sum_heat/(n_avg_heat-1);
					sum_heat=0;
					n_avg_heat=0;
					
					sendtemp(avg_heat);
				}	
				sendtemp(t_heat);//temporaneo test
				break;
			}

			
		}

		//temporaneo
		avg_cell=t_cell;
		avg_heat=t_heat;
		
		
		if(avg_cell>avg_heat)
		{
			RC0=1;
			RC1=0;
		}
		if(avg_heat>avg_cell)
		{
			RC0=0;
			RC1=1;
		}
		if(avg_heat==avg_cell)
		{
			RC0=1;
			RC1=1;
		}
		
		if((LO)&&(!LOenable)) //LO Signal
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
		
		if((SODS)&&(LOstate)&&(!SODSenable)) //SODS Signal
		{
			ABstate=1;
			if((SODS)&&(LOstate)&&(!SODSenable)&&(ABflag)) 
			{
				SODSstate=1;
				SODSenable=1;
				SODS_LED=1;
				ABflag=0;
				
				RA4=1;
			
			
			}
		}
		
		if((SOE)&&(SODSstate)&&(LOstate)&&(!SOEenable)) //SOE Signal
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
		tr_heat++;
		if((SOEstate)&&(tr_heat>=TR_HEATER))
		{
			if((avg_heat-avg_cell)>146)
			{
				HEATER=0;
			}
			else
			{
				if((avg_heat-avg_cell)>(146-(TLINEARE*15)))
				{
					HEATER=(avg_cell+146-avg_heat)*(255/(TLINEARE*15));
				}
				else
				{
					HEATER=255;
				}
			}
			tr_heat=0;
		}
	}*/
}

unsigned int bin_dec(unsigned int bin)
{
	unsigned int dec=0;

	for (int k=1;bin;k*=2,bin/=10)
	{
		dec+=(bin%10)*k;
	}
	return dec;
}


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

	
void DelayMillisec(int millisec)
{
	for(int i=0;i<millisec*4;i++)
	{
		DelayUs(250);
	}
}

void putch(unsigned char byte) 
{
	/* output one byte */
	while(!TXIF)	/* set when register is empty */
		continue;
	TXREG = byte;
}

unsigned char getch() 
{
	/* retrieve one byte */
	while(!RCIF)	/* set when register is not empty */
		continue;
	return RCREG;	
}

unsigned char getche(void)
{
	unsigned char c;
	putch(c = getch());
	return c;
}

void interrupt isr(void)
{
	if (TMR0IF) //Interrupt da TMR0
	{
		/* Reset TMR0 internal counter */
		TMR0H = T0_RELOAD_HIGH;
        TMR0L = T0_RELOAD_LOW;
		
		acquisition_data.time++; //Global time
		
		if(LOstate)
		{
			TimerLaser++;
		}
		
		if (TimerLaser>=TEMPOLASER)
		{
			LASER=1; //Accendo il laser
		}
		
		if(SOEstate)
		{
			//TimerHeater++; //per prove off
		}
		
		if (TimerHeater>=TEMPOHEATER)
		{
			HEATER=0; //Spegne heater
			SOEstate=0; //Disabilita il segnale SOE
		}
		
		TimerConv++;
		if (TimerConv>=TEMPOCONV)
		{
			conv=1; //Abilito conversione adc
			TimerConv=0;
		}
		
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
		
		/* Timer interrupt flag reset */
        TMR0IF = 0;
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
