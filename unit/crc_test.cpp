/**
 * @file crc_test.cpp
 * @brief Cyclic Redundancy Check algorithm unit testing. 
 */

#include "crc.h"

#include "gtests/gtest.h"

static uint8_t const data[24] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
                                       15, 16, 17, 18, 19, 20, 21, 22, 23}; 

TEST_F(CRCTest, CRCGeneration) {
    EXPECT_EQ(crc8(data, 24), DD);  
}

