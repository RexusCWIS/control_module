/**
 * @file i2c_frames.h
 * @brief Defines the first byte of data frames sent over I2C. 
 */

#ifndef DEF_I2C_FRAMES_H
#define DEF_I2C_FRAMES_H

#include <stdint.h>

typedef struct {
    uint16_t data;
}pressure_data_s;

typedef struct {
    uint16_t data;
}temperature_data_s; 

typedef enum {
    START_ACQUISITION = 'G', 
    STOP_ACQUISITION  = 'S'
} i2c_order_e; 

/**
 * @brief I2C data frame.
 * @details This frame represents the actual data sent over the I2C bus. 
 *          All data are filled at acquisition time and are read when 
 *          the I2C bus master performs a read operation. 
 */
typedef struct {
    uint32_t time;                      /**< @brief Time at which data acquisition occured. */
    temperature_data_s temperatures[3]; /**< @brief Temperature sensors data. */
    pressure_data_s pressure;           /**< @brief Pressure sensor data. */
    uint8_t status[2];                  /**< @brief Control module status. */
} i2c_frame_s;

typedef struct {
    uint16_t nb_of_images;
    uint8_t framerate;
    uint8_t status;
} i2c_camera_data_s;

typedef struct {
    i2c_frame_s acquired_data;
    i2c_camera_data_s camera_data;
} serial_frame_s; 

#endif  /* DEF_I2C_FRAMES_H */

