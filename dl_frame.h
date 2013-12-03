/**
 * @file dl_frame.h
 * @brief Represents the data frame used for downlink through the service module. 
 */

#ifndef DEF_DL_FRAME_H
#define DEF_DL_FRAME_H

#include "i2c_frames.h"

/**
 * @brief Structure representing the data frames sent by the experiment to the service module.
 */
typedef struct {
    unsigned char sync[2];              /**< @brief Synchronization bytes. */
    unsigned char msgID;                /**< @brief Message ID. */
    unsigned char msgCnt;               /**< @brief Message counter. */
    unsigned char time[4];              /**< @brief Time at which the message was sent. */
    TemperatureData_s temperature[3];   /**< @brief Temperature sensors data. */
    PressureData_s    pressure;         /**< @brief Pressure sensor data. */
    unsigned char reserved[4];          /**< @brief Padding bits. */
    unsigned char csm[2];               /**< @brief Checksum bytes. */ 
    unsigned char crc[2];               /**< @brief Cyclic redundancy checking bytes. */
}DownlinkFrame_s;

#endif  /* DEF_DL_FRAME_H */

