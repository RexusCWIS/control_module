opt subtitle "HI-TECH Software Omniscient Code Generator (PRO mode) build 11162"

opt pagewidth 120

	opt pm

	processor	18F2520
porta	equ	0F80h
portb	equ	0F81h
portc	equ	0F82h
portd	equ	0F83h
porte	equ	0F84h
lata	equ	0F89h
latb	equ	0F8Ah
latc	equ	0F8Bh
latd	equ	0F8Ch
late	equ	0F8Dh
trisa	equ	0F92h
trisb	equ	0F93h
trisc	equ	0F94h
trisd	equ	0F95h
trise	equ	0F96h
pie1	equ	0F9Dh
pir1	equ	0F9Eh
ipr1	equ	0F9Fh
pie2	equ	0FA0h
pir2	equ	0FA1h
ipr2	equ	0FA2h
t3con	equ	0FB1h
tmr3l	equ	0FB2h
tmr3h	equ	0FB3h
ccp1con	equ	0FBDh
ccpr1l	equ	0FBEh
ccpr1h	equ	0FBFh
adcon1	equ	0FC1h
adcon0	equ	0FC2h
adresl	equ	0FC3h
adresh	equ	0FC4h
sspcon2	equ	0FC5h
sspcon1	equ	0FC6h
sspstat	equ	0FC7h
sspadd	equ	0FC8h
sspbuf	equ	0FC9h
t2con	equ	0FCAh
pr2	equ	0FCBh
tmr2	equ	0FCCh
t1con	equ	0FCDh
tmr1l	equ	0FCEh
tmr1h	equ	0FCFh
rcon	equ	0FD0h
wdtcon	equ	0FD1h
lvdcon	equ	0FD2h
osccon	equ	0FD3h
t0con	equ	0FD5h
tmr0l	equ	0FD6h
tmr0h	equ	0FD7h
status	equ	0FD8h
fsr2	equ	0FD9h
fsr2l	equ	0FD9h
fsr2h	equ	0FDAh
plusw2	equ	0FDBh
preinc2	equ	0FDCh
postdec2	equ	0FDDh
postinc2	equ	0FDEh
indf2	equ	0FDFh
bsr	equ	0FE0h
fsr1	equ	0FE1h
fsr1l	equ	0FE1h
fsr1h	equ	0FE2h
plusw1	equ	0FE3h
preinc1	equ	0FE4h
postdec1	equ	0FE5h
postinc1	equ	0FE6h
indf1	equ	0FE7h
wreg	equ	0FE8h
fsr0	equ	0FE9h
fsr0l	equ	0FE9h
fsr0h	equ	0FEAh
plusw0	equ	0FEBh
preinc0	equ	0FECh
postdec0	equ	0FEDh
postinc0	equ	0FEEh
indf0	equ	0FEFh
intcon3	equ	0FF0h
intcon2	equ	0FF1h
intcon	equ	0FF2h
prod	equ	0FF3h
prodl	equ	0FF3h
prodh	equ	0FF4h
tablat	equ	0FF5h
tblptr	equ	0FF6h
tblptrl	equ	0FF6h
tblptrh	equ	0FF7h
tblptru	equ	0FF8h
pcl	equ	0FF9h
pclat	equ	0FFAh
pclath	equ	0FFAh
pclatu	equ	0FFBh
stkptr	equ	0FFCh
tosl	equ	0FFDh
tosh	equ	0FFEh
tosu	equ	0FFFh
skipnz macro
	btfsc	status,2
	endm
	global	__ramtop
	global	__accesstop
# 19 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PORTA equ 0F80h ;# 
# 90 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PORTB equ 0F81h ;# 
# 139 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PORTC equ 0F82h ;# 
# 192 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PORTE equ 0F84h ;# 
# 317 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
LATA equ 0F89h ;# 
# 370 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
LATB equ 0F8Ah ;# 
# 423 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
LATC equ 0F8Bh ;# 
# 476 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TRISA equ 0F92h ;# 
# 481 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
DDRA equ 0F92h ;# 
# 539 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TRISB equ 0F93h ;# 
# 544 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
DDRB equ 0F93h ;# 
# 602 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TRISC equ 0F94h ;# 
# 607 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
DDRC equ 0F94h ;# 
# 665 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
OSCTUNE equ 0F9Bh ;# 
# 687 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PIE1 equ 0F9Dh ;# 
# 713 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PIR1 equ 0F9Eh ;# 
# 747 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
IPR1 equ 0F9Fh ;# 
# 773 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PIE2 equ 0FA0h ;# 
# 796 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PIR2 equ 0FA1h ;# 
# 819 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
IPR2 equ 0FA2h ;# 
# 842 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
EECON1 equ 0FA6h ;# 
# 865 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
EECON2 equ 0FA7h ;# 
# 877 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
EEDATA equ 0FA8h ;# 
# 889 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
EEADR equ 0FA9h ;# 
# 901 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
RCSTA equ 0FABh ;# 
# 906 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
RCSTA1 equ 0FABh ;# 
# 954 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TXSTA equ 0FACh ;# 
# 959 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TXSTA1 equ 0FACh ;# 
# 1053 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TXREG equ 0FADh ;# 
# 1058 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TXREG1 equ 0FADh ;# 
# 1076 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
RCREG equ 0FAEh ;# 
# 1081 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
RCREG1 equ 0FAEh ;# 
# 1099 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
SPBRG equ 0FAFh ;# 
# 1104 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
SPBRG1 equ 0FAFh ;# 
# 1122 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
SPBRGH equ 0FB0h ;# 
# 1134 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
T3CON equ 0FB1h ;# 
# 1175 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TMR3 equ 0FB2h ;# 
# 1187 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TMR3L equ 0FB2h ;# 
# 1199 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TMR3H equ 0FB3h ;# 
# 1211 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
CMCON equ 0FB4h ;# 
# 1244 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
CVRCON equ 0FB5h ;# 
# 1270 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
ECCP1AS equ 0FB6h ;# 
# 1275 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
ECCPAS equ 0FB6h ;# 
# 1315 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PWM1CON equ 0FB7h ;# 
# 1320 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
ECCP1DEL equ 0FB7h ;# 
# 1340 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
BAUDCON equ 0FB8h ;# 
# 1345 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
BAUDCTL equ 0FB8h ;# 
# 1405 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
CCP2CON equ 0FBAh ;# 
# 1431 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
CCPR2 equ 0FBBh ;# 
# 1443 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
CCPR2L equ 0FBBh ;# 
# 1455 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
CCPR2H equ 0FBCh ;# 
# 1467 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
CCP1CON equ 0FBDh ;# 
# 1493 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
CCPR1 equ 0FBEh ;# 
# 1505 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
CCPR1L equ 0FBEh ;# 
# 1517 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
CCPR1H equ 0FBFh ;# 
# 1529 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
ADCON2 equ 0FC0h ;# 
# 1552 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
ADCON1 equ 0FC1h ;# 
# 1585 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
ADCON0 equ 0FC2h ;# 
# 1640 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
ADRES equ 0FC3h ;# 
# 1652 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
ADRESL equ 0FC3h ;# 
# 1664 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
ADRESH equ 0FC4h ;# 
# 1676 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
SSPCON2 equ 0FC5h ;# 
# 1703 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
SSPCON1 equ 0FC6h ;# 
# 1725 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
SSPSTAT equ 0FC7h ;# 
# 1832 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
SSPADD equ 0FC8h ;# 
# 1844 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
SSPBUF equ 0FC9h ;# 
# 1856 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
T2CON equ 0FCAh ;# 
# 1886 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PR2 equ 0FCBh ;# 
# 1891 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
MEMCON equ 0FCBh ;# 
# 1909 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TMR2 equ 0FCCh ;# 
# 1921 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
T1CON equ 0FCDh ;# 
# 1958 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TMR1 equ 0FCEh ;# 
# 1970 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TMR1L equ 0FCEh ;# 
# 1982 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TMR1H equ 0FCFh ;# 
# 1994 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
RCON equ 0FD0h ;# 
# 2039 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
WDTCON equ 0FD1h ;# 
# 2054 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
HLVDCON equ 0FD2h ;# 
# 2059 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
LVDCON equ 0FD2h ;# 
# 2129 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
OSCCON equ 0FD3h ;# 
# 2154 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
T0CON equ 0FD5h ;# 
# 2179 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TMR0 equ 0FD6h ;# 
# 2191 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TMR0L equ 0FD6h ;# 
# 2203 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TMR0H equ 0FD7h ;# 
# 2215 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
STATUS equ 0FD8h ;# 
# 2246 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
FSR2 equ 0FD9h ;# 
# 2258 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
FSR2L equ 0FD9h ;# 
# 2270 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
FSR2H equ 0FDAh ;# 
# 2282 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PLUSW2 equ 0FDBh ;# 
# 2294 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PREINC2 equ 0FDCh ;# 
# 2306 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
POSTDEC2 equ 0FDDh ;# 
# 2318 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
POSTINC2 equ 0FDEh ;# 
# 2330 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
INDF2 equ 0FDFh ;# 
# 2342 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
BSR equ 0FE0h ;# 
# 2354 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
FSR1 equ 0FE1h ;# 
# 2366 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
FSR1L equ 0FE1h ;# 
# 2378 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
FSR1H equ 0FE2h ;# 
# 2390 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PLUSW1 equ 0FE3h ;# 
# 2402 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PREINC1 equ 0FE4h ;# 
# 2414 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
POSTDEC1 equ 0FE5h ;# 
# 2426 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
POSTINC1 equ 0FE6h ;# 
# 2438 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
INDF1 equ 0FE7h ;# 
# 2450 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
WREG equ 0FE8h ;# 
# 2473 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
FSR0 equ 0FE9h ;# 
# 2485 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
FSR0L equ 0FE9h ;# 
# 2497 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
FSR0H equ 0FEAh ;# 
# 2509 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PLUSW0 equ 0FEBh ;# 
# 2521 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PREINC0 equ 0FECh ;# 
# 2533 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
POSTDEC0 equ 0FEDh ;# 
# 2545 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
POSTINC0 equ 0FEEh ;# 
# 2557 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
INDF0 equ 0FEFh ;# 
# 2569 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
INTCON3 equ 0FF0h ;# 
# 2598 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
INTCON2 equ 0FF1h ;# 
# 2625 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
INTCON equ 0FF2h ;# 
# 2689 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PROD equ 0FF3h ;# 
# 2701 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PRODL equ 0FF3h ;# 
# 2713 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PRODH equ 0FF4h ;# 
# 2725 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TABLAT equ 0FF5h ;# 
# 2737 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TBLPTR equ 0FF6h ;# 
# 2750 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TBLPTRL equ 0FF6h ;# 
# 2762 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TBLPTRH equ 0FF7h ;# 
# 2774 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TBLPTRU equ 0FF8h ;# 
# 2787 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PCLAT equ 0FF9h ;# 
# 2792 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PC equ 0FF9h ;# 
# 2810 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PCL equ 0FF9h ;# 
# 2822 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PCLATH equ 0FFAh ;# 
# 2834 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
PCLATU equ 0FFBh ;# 
# 2846 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
STKPTR equ 0FFCh ;# 
# 2870 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TOS equ 0FFDh ;# 
# 2882 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TOSL equ 0FFDh ;# 
# 2894 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TOSH equ 0FFEh ;# 
# 2906 "C:\Program Files (x86)\HI-TECH Software\PICC-18\9.80\include\pic18f2520.h"
TOSU equ 0FFFh ;# 
	FNCALL	_main,_settings
	FNROOT	_main
	global	_ADCON1
_ADCON1	set	0xFC1
	global	_ADCON2
_ADCON2	set	0xFC0
	global	_BAUDCON
_BAUDCON	set	0xFB8
	global	_CCP1CON
_CCP1CON	set	0xFBD
	global	_CCPR1L
_CCPR1L	set	0xFBE
	global	_CCPR2L
_CCPR2L	set	0xFBB
	global	_PORTA
_PORTA	set	0xF80
	global	_PORTB
_PORTB	set	0xF81
	global	_PORTC
_PORTC	set	0xF82
	global	_PR2
_PR2	set	0xFCB
	global	_RCSTA
_RCSTA	set	0xFAB
	global	_T2CON
_T2CON	set	0xFCA
	global	_TRISA
_TRISA	set	0xF92
	global	_TRISB
_TRISB	set	0xF93
	global	_TRISC
_TRISC	set	0xF94
	global	_TXSTA
_TXSTA	set	0xFAC
	global	_GIE
_GIE	set	0x7F97
	global	_PEIE
_PEIE	set	0x7F96
	global	_RCIE
_RCIE	set	0x7CED
	global	_SPEN
_SPEN	set	0x7D5F
psect	text0,class=CODE,space=0,reloc=2
global __ptext0
__ptext0:
; #config settings
	file	"rexus_micro_v1.as"
	line	#
psect	cinit,class=CODE,delta=1,reloc=2
global __pcinit
__pcinit:
global start_initialization
start_initialization:

psect cinit,class=CODE,delta=1
global end_of_initialization

;End of C runtime variable initialization code

end_of_initialization:
movlb 0
goto _main	;jump to C main() function
psect	cstackCOMRAM,class=COMRAM,space=1
global __pcstackCOMRAM
__pcstackCOMRAM:
	global	?_settings
?_settings:	; 0 bytes @ 0x0
	global	??_settings
??_settings:	; 0 bytes @ 0x0
	global	?_main
?_main:	; 0 bytes @ 0x0
	global	??_main
??_main:	; 0 bytes @ 0x0
;!
;!Data Sizes:
;!    Strings     0
;!    Constant    0
;!    Data        0
;!    BSS         0
;!    Persistent  0
;!    Stack       0
;!
;!Auto Spaces:
;!    Space          Size  Autos    Used
;!    COMRAM          127      0       0
;!    BANK0           128      0       0
;!    BANK1           256      0       0
;!    BANK2           256      0       0
;!    BANK3           256      0       0
;!    BANK4           256      0       0
;!    BANK5           256      0       0

;!
;!Pointer List with Targets:
;!
;!    None.


;!
;!Critical Paths under _main in COMRAM
;!
;!    None.
;!
;!Critical Paths under _main in BANK0
;!
;!    None.
;!
;!Critical Paths under _main in BANK1
;!
;!    None.
;!
;!Critical Paths under _main in BANK2
;!
;!    None.
;!
;!Critical Paths under _main in BANK3
;!
;!    None.
;!
;!Critical Paths under _main in BANK4
;!
;!    None.
;!
;!Critical Paths under _main in BANK5
;!
;!    None.

;;
;;Main: autosize = 0, tempsize = 0, incstack = 0, save=0
;;

;!
;!Call Graph Tables:
;!
;! ---------------------------------------------------------------------------------
;! (Depth) Function   	        Calls       Base Space   Used Autos Params    Refs
;! ---------------------------------------------------------------------------------
;! (0) _main                                                 0     0      0       0
;!                           _settings
;! ---------------------------------------------------------------------------------
;! (1) _settings                                             0     0      0       0
;! ---------------------------------------------------------------------------------
;! Estimated maximum stack depth 1
;! ---------------------------------------------------------------------------------
;!
;! Call Graph Graphs:
;!
;! _main (ROOT)
;!   _settings
;!

;! Address spaces:

;!Name               Size   Autos  Total    Cost      Usage
;!BITCOMRAM           7F      0       0       0        0.0%
;!EEDATA             100      0       0       0        0.0%
;!NULL                 0      0       0       0        0.0%
;!CODE                 0      0       0       0        0.0%
;!COMRAM              7F      0       0       1        0.0%
;!STACK                0      0       1       2        0.0%
;!DATA                 0      0       0       3        0.0%
;!BITBANK0            80      0       0       4        0.0%
;!BANK0               80      0       0       5        0.0%
;!BITBANK1           100      0       0       6        0.0%
;!BANK1              100      0       0       7        0.0%
;!BITBANK2           100      0       0       8        0.0%
;!BANK2              100      0       0       9        0.0%
;!BITBANK3           100      0       0      10        0.0%
;!BANK3              100      0       0      11        0.0%
;!BITBANK4           100      0       0      12        0.0%
;!BANK4              100      0       0      13        0.0%
;!BANK5              100      0       0      14        0.0%
;!ABS                  0      0       0      15        0.0%
;!BITBANK5           100      0       0      16        0.0%
;!BIGRAM             5FF      0       0      17        0.0%
;!BITSFR               0      0       0      40        0.0%
;!SFR                  0      0       0      40        0.0%

	global	_main

;; *************** function _main *****************
;; Defined at:
;;		line 22 in file "D:\Fabrizio\REXUS Project\firmware\c\rexus_micro_v1.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: FFFFFFFF/0
;; Data sizes:     COMRAM   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5
;;      Params:         0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels required when called:    1
;; This function calls:
;;		_settings
;; This function is called by:
;;		Startup code after reset
;; This function uses a non-reentrant model
;;
psect	text0
psect	text0
	file	"D:\Fabrizio\REXUS Project\firmware\c\rexus_micro_v1.c"
	line	22
	global	__size_of_main
	__size_of_main	equ	__end_of_main-_main
	
_main:
	opt	stack 30
	line	23
	
l657:
;rexus_micro_v1.c: 23: settings();
	call	_settings	;wreg free
	line	26
	
l56:
	global	start
	goto	start
	opt stack 0
GLOBAL	__end_of_main
	__end_of_main:
	signat	_main,88
	global	_settings

;; *************** function _settings *****************
;; Defined at:
;;		line 25 in file "D:\Fabrizio\REXUS Project\firmware\c\settings.h"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: FFFFFFFF/0
;; Data sizes:     COMRAM   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5
;;      Params:         0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1,class=CODE,space=0,reloc=2
global __ptext1
__ptext1:
psect	text1
	file	"D:\Fabrizio\REXUS Project\firmware\c\settings.h"
	line	25
	global	__size_of_settings
	__size_of_settings	equ	__end_of_settings-_settings
	
_settings:
	opt	stack 30
	line	27
	
l635:
;settings.h: 27: PORTA=0;
	clrf	((c:3968)),c	;volatile
	line	28
	
l637:
;settings.h: 28: TRISA=0b00000111;
	movlw	low(07h)
	movwf	((c:3986)),c	;volatile
	line	29
	
l639:
;settings.h: 29: PORTB=0;
	clrf	((c:3969)),c	;volatile
	line	30
	
l641:
;settings.h: 30: TRISB=0b00000000;
	clrf	((c:3987)),c	;volatile
	line	31
	
l643:
;settings.h: 31: PORTC=0;
	clrf	((c:3970)),c	;volatile
	line	32
;settings.h: 32: TRISC=0b11000000;
	movlw	low(0C0h)
	movwf	((c:3988)),c	;volatile
	line	34
	
l645:
;settings.h: 34: GIE=1;
	bsf	c:(32663/8),(32663)&7	;volatile
	line	35
	
l647:
;settings.h: 35: PEIE=1;
	bsf	c:(32662/8),(32662)&7	;volatile
	line	36
	
l649:
;settings.h: 36: RCIE=1;
	bsf	c:(31981/8),(31981)&7	;volatile
	line	39
	
l651:
;settings.h: 39: SPEN=1;
	bsf	c:(32095/8),(32095)&7	;volatile
	line	40
;settings.h: 40: TXSTA=0b00100110;
	movlw	low(026h)
	movwf	((c:4012)),c	;volatile
	line	41
;settings.h: 41: RCSTA=0b10010000;
	movlw	low(090h)
	movwf	((c:4011)),c	;volatile
	line	42
	
l653:
;settings.h: 42: BAUDCON=0b00000000;
	clrf	((c:4024)),c	;volatile
	line	45
;settings.h: 45: ADCON1=0b00001100;
	movlw	low(0Ch)
	movwf	((c:4033)),c	;volatile
	line	46
;settings.h: 46: ADCON2=0b10010100;
	movlw	low(094h)
	movwf	((c:4032)),c	;volatile
	line	49
	
l655:
;settings.h: 49: PR2=0b11111111;
	setf	((c:4043)),c	;volatile
	line	50
;settings.h: 50: CCPR1L=128;
	movlw	low(080h)
	movwf	((c:4030)),c	;volatile
	line	51
;settings.h: 51: CCPR2L=128;
	movlw	low(080h)
	movwf	((c:4027)),c	;volatile
	line	52
;settings.h: 52: T2CON=0b100;
	movlw	low(04h)
	movwf	((c:4042)),c	;volatile
	line	53
;settings.h: 53: CCP1CON=0b00001100;
	movlw	low(0Ch)
	movwf	((c:4029)),c	;volatile
	line	54
;settings.h: 54: CCP1CON=0b00001100;
	movlw	low(0Ch)
	movwf	((c:4029)),c	;volatile
	line	55
	
l43:
	return
	opt stack 0
GLOBAL	__end_of_settings
	__end_of_settings:
	signat	_settings,88
	GLOBAL	__activetblptr
__activetblptr	EQU	0
	psect	intsave_regs,class=BIGRAM,space=1
psect	text2,class=CODE,space=0,reloc=2
global __ptext2
__ptext2:
	PSECT	rparam,class=COMRAM,space=1
	GLOBAL	__Lrparam
	FNCONF	rparam,??,?
GLOBAL	__Lparam, __Hparam
GLOBAL	__Lrparam, __Hrparam
__Lparam	EQU	__Lrparam
__Hparam	EQU	__Hrparam
	end
