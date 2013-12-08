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

int LOstate=0, SODSstate=0, SOEstate=0, ABstate=0, ABflag=0, conv=0;
int TimerLaser=0, TimerHeater=0, TimerConv=0, TimerAB=0;

void main(void)
{
	

	int LOenable=0, SODSenable=0, SOEenable=0, chconv=0, n_avg_cell=0, n_avg_heat=0;
	unsigned int t_cell=0, t_heat=0, sum_cell=0, sum_heat=0, avg_cell=0, avg_heat=0, tr_heat=0;
	settings(); // Imposto le porte come da settings.h	
	
	
	
	
	
	
	
	
	while(1)
	{
		if (conv)
		{
			ADCON0=SENSOR0;
			GO=1;
			while(GO)
			t_cell=(ADRESH<<8)+ADRESL;
			
			t_cell=bin_dec(t_cell); //*0.8 
			
			while(!TXIF)
			continue;
			TXREG=t_cell;
			
			
			
			
			ADCON0=SENSOR1;
			GO=1;
			while(GO)
			t_heat=(ADRESH<<8)+ADRESL;
			
			t_heat=bin_dec(t_heat);
			
			while(!TXIF)
			continue;
			TXREG=t_heat;
			
			
			//sendtemp(t_heat);
			
			
			
			avg_cell=t_cell;
			avg_heat=t_heat;
			
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
		TMR0=100; //Reimposto TMR0
		
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
		
		TMR0IF=0; //Resetto il flag interrupt TMR0
	}
}