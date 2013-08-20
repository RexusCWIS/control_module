/*-------- REXUS Project -------*\
|*-----CWIS Microcontroller-----*|
|*------------main.c------------*|
|*----------PIC18F2520----------*|
\*--Versione 1.0.0 - Gen. 2013--*/

#include "pic18f2520 configbits.h"
#include<htc.h>
#define _LEGACY_HEADERS 
#define XTAL_FREQ 20MHZ


#include "settings.h"
#include "delay.h"
#include "usart.c"

int LOstate=0, SODSstate=0, SOEstate=0;
int TimerLaser=0, TimerHeater=0;

void main(void)
{
	

	int state=0, LOenable=0, SODSenable=0, SOEenable=0;
	char variabile, a, temp[6];
	settings(); // Imposto le porte come da settings.h	
	
	while(1)
	{		
		if((LO)&&(!LOenable)) //LO Signal
		{
			DelayMs(50);
			if((LO)&&(!LOenable)) 
			{
			LOstate=1;
			LOenable=1;
			}
		}
		
		if((SODS)&&(LO)&&(!SODSenable)) //SODS Signal
		{
			DelayMs(50);
			if((SODS)&&(LO)&&(!SODSenable)) 
			{
			SODSstate=1;
			SODSenable=1;
			}
		}
		
		if((SOE)&&(SODS)&&(LO)&&(!SOEenable)) //SOE Signal
		{
			DelayMs(50);
			if((SOE)&&(SODS)&&(LO)&&(!SOEenable)) 
			{
			SOEstate=1;
			SOEenable=1;
			}
		}
		if(SOEstate)
		{
			HEATER=255; //Solo per test
			/* //Solo con sensori funzionanti
			if(TempCell+9>TempHeater)
			{
				HEATER=255;
			}
			else
			{
				if(TempCell+10>TempHeater)
				{
					HEATER=(TempCell+10-TempHeater)*255;
				}
			}*/
		}
		
		/*
		if((RB3)&&(HEATER<255))
		{
			DelayMs(100);
			if(RB3)
			{
				HEATER++;
			}
		}
		if((RB2)&&(HEATER>0))
		{
			DelayMs(100);
			if(RB2)
			{
				HEATER--;
			}
		}*/
	}
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
			TimerHeater++;
		}
		if (TimerHeater>=TEMPOHEATER)
		{
			HEATER=0; //Spegne heater
			SOEstate=0; //Disabilita il segnale SOE
		}
		TMR0IF=0; //Resetto il flag interrupt TMR0
	}
}