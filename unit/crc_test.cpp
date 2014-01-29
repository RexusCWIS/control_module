/**
 * @file crc_test.cpp
 * @brief Cyclic Redundancy Check algorithm unit testing. 
 */

#include "libs/crc.h"

#include "gtest/gtest.h"

static unsigned char const data[24] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                 15, 16, 17, 18, 19, 20, 21, 22, 23}; 

TEST(CRCTest, CRCGeneration) {
    EXPECT_EQ(crc(data, 24), 0x346B);  
}
/**
 * @brief Main unit tests control function. 
 * @details Launches the unit tests and controls the output formats. 
 */
int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS(); 
}


