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



void main(void)
{
	
	int state=0;
	char variabile, a, temp[6];
	settings(); // Imposto le porte come da settings.h	
	
	switch(state)
	{
		case 0: //Il sistema è in standby e attende di essere avviato
		printf("STBY"); //standby
		if(RCIF)
		{
			printf("RX");
			if(RCREG=='A')
			{
				state=1;
			}
		}
		DelayUs(250);
		break;
		
		case 1:
		printf("ACT");
		
		
		

	while(1)
	{		
		read_ad(0, temp);
		printf("T0 = ");
		printf(temp);
		printf(" ");
		
		read_ad(1, temp);
		printf("T1 = ");
		printf(temp);
		printf(" ");
		
		read_ad(2, temp);
		printf("T2 = ");
		printf(temp);
		printf("\n\r");
		
		
		
	}
	

}



void read_ad (int n_sensor, char ad_value[6])
	{
		int decimal, celsius=0;
		switch(n_sensor)
		{
			case 0: //rilevazione e conversione sorgente AN0
			ADCON0=SENSOR0;
			//DelayUs(10);
			GO=1;
			while(GO) //attesa conversione
			continue;
			break;
			
			case 1: //rilevazione e conversione sorgente AN1
			ADCON0=SENSOR1;
			//DelayUs(10);
			GO=1;
			while(GO) //attesa conversione
			continue;
			break;
			
			case 2: //rilevazione e conversione sorgente AN2
			ADCON0=SENSOR2;
			//DelayUs(10);
			GO=1;
			while(GO) //attesa conversione
			continue;
			break;
		}
		//celsius=(ADRESH<<8)+ADRESL;
		//celsius=((celsius/3)*4,882)-500;
		celsius=(((((ADRESH<<8)+ADRESL)/3)*4,882))-500;
		
		if(celsius<0)
		{
			celsius=celsius*(-1);
			ad_value[0]='-';
		}
		else
		{
			ad_value[0]='+';
		}
		ad_value[1]=((celsius/1000)%10)+'0';
		ad_value[2]=((celsius/100)%10)+'0';
		ad_value[3]=((celsius/10)%10)+'0';
		ad_value[4]=',';
		ad_value[5]=(celsius%10)+'0';			
	}

	
	
	
	
	
	
	
	
	
	
void DelayMs(int millisec)
{
	for(int i=0;i<millisec*4;i++)
	{
		DelayUs(250);
	}
}
	
/*	

temp=ADRESH;
while(temp>0)
{
	temp=temp/10;
	i++;
}
temp=ADRESH;
for(int j=i-1;j>=0;j--)
{
	ad_value[j]=(temp%10)+'0';
	temp=temp/10;
}*/

//eeprom_write(0, ADRESH);
//eeprom_write(1, ADRESL);	