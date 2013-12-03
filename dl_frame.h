/**
 * @file dl_frame.h
 * @brief Represents the data frame used for downlink through the service module. 
 */

#ifndef DEF_DL_FRAME_H
#define DEF_DL_FRAME_H

#include "i2c_frames.h"

typedef struct {
    unsigned char sync[2];
    unsigned char msgID;
    unsigned char msgCnt;
    unsigned char time[4];
    TemperatureData_s temperature[3];
    PressureData_s    pressure; 
    unsigned char reserved[4]; 
    unsigned char csm[2]; 
    unsigned char crc[2]; 
}DownlinkFrame_s;

#endif  /* DEF_DL_FRAME_H */

