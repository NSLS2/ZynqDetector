/**
 * @file PsI2c.hpp
 * @brief Class definition of `PsI2c` — Linux version.
 * @details
 * I2C bus access via /dev/i2c-N + ioctl.
 * Replaces Xilinx XIicPs driver.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <cstdint>
#include <mutex>

#include "Logger.hpp"

//===========================================================================//

class PsI2c
{
public:
    PsI2c( uint8_t bus_index, const Logger& logger );
    ~PsI2c();

    PsI2c(const PsI2c&) = delete;
    PsI2c& operator=(const PsI2c&) = delete;

    /**
     * @brief Write data to an I2C slave.
     * @param slave_addr  7-bit I2C address.
     * @param buffer      Data to write.
     * @param length      Number of bytes.
     * @return 0 on success, -1 on failure.
     */
    int write( uint8_t slave_addr, const uint8_t* buffer, uint16_t length );

    /**
     * @brief Read data from an I2C slave.
     * @param slave_addr  7-bit I2C address.
     * @param buffer      Buffer to read into.
     * @param length      Number of bytes.
     * @return 0 on success, -1 on failure.
     */
    int read( uint8_t slave_addr, uint8_t* buffer, uint16_t length );

private:
#ifndef SIM_MODE
    int            fd_;
    std::mutex     mutex_;
#endif
    uint8_t        bus_index_;
    const Logger&  logger_;
};

//===========================================================================//

