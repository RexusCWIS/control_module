/**
 * @file crc_test.cpp
 * @brief Cyclic Redundancy Check algorithm unit testing. 
 */

#include "libs/crc.h"

#include "crc16.h"
#include "gtest/gtest.h"

static unsigned char data[26] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                 15, 16, 17, 18, 19, 20, 21, 22, 23, 0, 0}; 


TEST(CRCTest, TableGeneration) {
    for(int index = 0; index < 256; index++) {
        ASSERT_EQ(crc_table[index], crc16_tbl[index]);  
    }
}

/**
 * @brief Compares the results between an open source CRC algorithm and ours. 
 */
TEST(CRCTest, CRCGeneration) {
    EXPECT_EQ(crc(data, 24), crc16_block(0xFFFFu, data, 24));     
}

TEST(CRCTest, CRCTransmission) {

    uint16_t c = crc(data, 24); 
    data[24]   = (uint8_t) (c >> 8);
    data[25]   = (uint8_t) (c & 0xFFu); 

    EXPECT_EQ(crc(data, 26), 0);
}
/**
 * @brief Main unit tests control function. 
 * @details Launches the unit tests and controls the output formats. 
 */
int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS(); 
}


