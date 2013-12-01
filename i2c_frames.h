/**
 * @file i2c_frames.h
 * @brief Defines the first byte of data frames sent over I2C. 
 */

#ifndef DEF_I2C_FRAMES_H
#define DEF_I2C_FRAMES_H

/** @brief Specifies that the data frame contains values acquired by the sensors. */
#define DATA    0x0
/** @brief Specifies that the data frame contains orders to the acquisition board. */
#define ORDER   0xFF


#endif  /* DEF_I2C_FRAMES_H */

