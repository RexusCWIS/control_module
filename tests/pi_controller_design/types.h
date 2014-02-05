/**
 * @file types.h
 * @brief Types used for the PI controller design.
 */

#ifndef DEF_TYPES_H
#define DEF_TYPES_H

#include <stdint.h>

/** @brief Indicates that the module is ON. */
#define STATUS_POWER_ON         (1 << 0)
/** @brief Indicates a deadline miss for the ADC conversion. */
#define STATUS_OUT_OF_SYNC      (1 << 4)

typedef struct {
    uint8_t  sync[2];
    uint32_t time;
    uint16_t voltage;
    uint16_t temperature;
    uint8_t  status;
    uint8_t  crc[2];
} serial_frame_s;

#endif /* DEF_TYPES_H */
