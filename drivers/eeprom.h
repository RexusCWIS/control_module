/**
 * @file eeprom.h
 * @brief Data EEPROM access functions.
 */

#ifndef DEF_EEPROM_H
#define DEF_EEPROM_H

#include <htc.h>
#include <stdint.h>

/**
 * @brief Reads a @p data byte from the given @p address in the data EEPROM.
 * @param[in] address   Address from which the data must be read from.
 * @returns The @p data byte at the given @p address.
 * @see eeprom_write_byte()
 */
uint8_t eeprom_read_byte(uint8_t address);

/**
 * @brief Writes the given @p data byte to the given @p address in the data EEPROM.
 * @details Initiates a write sequence to the data EEPROM. This function returns
 *          once the write operation has been completed. A critical section is
 *          entered during the execution of this function as a specific sequence
 *          must be written to initiate the write operation. 
 * @param[in] data      Byte to be written in data EEPROM.
 * @param[in] address   Address at which the byte must be written.
 * @note This function does not perform any write verification. The user must
 *       perform it himself by calling the @ref eeprom_read_byte function
 *       and compare the data in memory with the data byte in RAM.
 * @see eeprom_read_byte()
 */
void eeprom_write_byte(uint8_t data, uint8_t address);

void eeprom_read_array(uint8_t data[], uint8_t size, uint8_t address);
void eeprom_write_array(uint8_t data[], uint8_t size, uint8_t address);

#endif /* DEF_EEPROM_H */
        