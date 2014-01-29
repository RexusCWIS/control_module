/**
 * @file crc.h
 * @brief Cyclic Redundancy Check algorithm function. 
 */

#ifndef DEF_CRC_H
#define DEF_CRC_H

#include <stdint.h>

typedef uint16_t crc_t;

#define WIDTH   (8 * sizeof(crc_t))

extern const crc_t crc_table[256]; 

static inline crc_t crc(uint8_t const msg[], uint8_t size) {

    uint8_t data; 
    crc_t remainder = 0; 

    for(uint8_t byte = 0; byte < size; byte++) {
        
        data = msg[byte] ^ (remainder >> (WIDTH - 8)); 
        remainder = crc_table[data] ^ (remainder << 8); 

    }
    return remainder; 
}

#endif  /* DEF_CRC_H */

