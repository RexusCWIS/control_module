/**
 * @file crc.h
 * @brief Cyclic Redundancy Check algorithm implementation. 
 */

#include "libs/crc.h"

#ifdef _cplusplus
extern "C" {
#endif  /* _cplusplus */

extern const crc_t crc_table[256]; 

crc_t crc(uint8_t const msg[], uint8_t size) {

    uint8_t data; 
    uint8_t index;

    crc_t remainder = INITIAL_REMAINDER; 

    for(index = 0; index < size; index++) {
        
        data = msg[index] ^ (remainder >> (WIDTH - 8)); 
        remainder = crc_table[data] ^ (remainder << 8); 
    }
    return remainder; 
}

#ifdef _cplusplus
}
#endif  /* _cplusplus */

