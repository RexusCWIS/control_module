/**
 * @file crc.h
 * @brief Cyclic Redundancy Check algorithm functions. 
 */

#ifndef DEF_CRC_H
#define DEF_CRC_H

#include <stdint.h>

typedef uint16_t crc_t;

#define WIDTH   (8 * sizeof(crc_t))
#define INITIAL_REMAINDER   0xFFFFu

crc_t crc(uint8_t const msg[], uint8_t size); 

#endif  /* DEF_CRC_H */

