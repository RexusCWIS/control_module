/**
 * @file types.h
 * @brief Definition of types and structures used in the application.
 */

#ifndef DEF_TYPES_H
#define DEF_TYPES_H

#include <stdint.h>

#define LO_DEBOUNCE_REQUEST     (1 << 0)
#define SODS_DEBOUNCE_REQUEST   (1 << 1)
#define SOE_DEBOUNCE_REQUEST    (1 << 2)
#define LO_DEBOUNCED_STATE      (1 << 4)
#define SODS_DEBOUNCED_STATE    (1 << 5)
#define SOE_DEBOUNCED_STATE     (1 << 6)

typedef enum {
    BOOT = 0,
    LO   = 1,
    SODS = 2,
    SOE  = 3,
    END  = 4
} experiment_state_e;

typedef enum {
    START_ACQUISITION = 'G', 
    STOP_ACQUISITION  = 'S'
} i2c_order_e; 

#define STATUS_POWER_ON     (1 << 0)
#define STATUS_LASER_ON     (1 << 1)
#define STATUS_CAMERA_ON    (1 << 2)
#define STATUS_HEATER_ON    (1 << 3)
#define STATUS_LO           (1 << 4)
#define STATUS_SODS         (1 << 5)
#define STATUS_SOE          (1 << 6)

/**
 * @brief I2C data frame.
 * @details This frame represents the actual data sent over the I2C bus. 
 *          All data are filled at acquisition time and are read when 
 *          the I2C bus master performs a read operation. 
 */
typedef struct {
    uint32_t time;                      /**< @brief Time at which data acquisition occured. */
    uint16_t temperatures[3];           /**< @brief Temperature sensors data. */
    uint16_t pressure;                  /**< @brief Pressure sensor data. */
    uint8_t heating;
    uint8_t status;                     /**< @brief Control module status. */
} i2c_frame_s;

typedef struct {
    uint16_t nb_of_images;
    uint8_t framerate;
    uint8_t status;
} i2c_camera_data_s;

typedef struct {
    uint8_t sync[2];
    i2c_frame_s acquired_data;
    i2c_camera_data_s camera_data;
    uint8_t checksum[2];
    uint8_t crc[2];
} serial_frame_s; 

#endif  /* DEF_TYPES_H */

