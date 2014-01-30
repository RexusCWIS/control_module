/**
 * @file crc.h
 * @brief Cyclic Redundancy Check algorithm functions. 
 */

#ifndef DEF_CRC_H
#define DEF_CRC_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#include <stdint.h>

typedef uint16_t crc_t;

#define WIDTH   (8 * sizeof(crc_t))
#define INITIAL_REMAINDER   0xFFFFu

crc_t crc(uint8_t const msg[], uint8_t size); 

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* DEF_CRC_H */

