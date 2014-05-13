/**
 * @file eeprom.c
 * @brief Implementation of data EEPROM access functions.
 */

#include "drivers/eeprom.h"

uint8_t eeprom_read_byte(uint8_t address) {

    EEADR = address;
    /* Access data EEPROM */
    EECON1bits.EEPGD = 0;
    /* Initiate read */
    EECON1bits.RD    = 1;

    /* Data is available the next cycle in EEDATA */
    return EEDATA;
}



void eeprom_write_byte(uint8_t data, uint8_t address) {
    
    EEADR  = address;
    EEDATA = data;

    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS  = 0;
    EECON1bits.WREN  = 1;

    /* Initiation of the write sequence */
    di();
    EECON2 = 0x55u;
    EECON2 = 0xAAu;
    EECON1bits.WR = 1;
    ei();

    /* Wait for the completion of the write operation */
    while(!PIR2bits.EEIF)
        ;

    /* Reset interrupt and write enable bits */
    PIR2bits.EEIF   = 0;
    EECON1bits.WREN = 0;
}

void eeprom_read_array(uint8_t data[], uint8_t size, uint8_t address) {

    for(uint8_t index = 0; index < size; index++) {
        data[index] = eeprom_read_byte(address + index);
    }
}


void eeprom_write_array(uint8_t data[], uint8_t size, uint8_t address) {

    for(uint8_t index = 0; index < size; index++) {
        eeprom_write_byte(data[index], address + index);
    }
}