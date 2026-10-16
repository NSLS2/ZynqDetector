/**
 * @file PsI2c.cpp
 * @brief Member function definitions of `PsI2c` — Linux version.
 * @details
 * Hardware build: /dev/i2c-N with ioctl I2C_RDWR for bus access.
 * Simulation build: no-op stubs.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "PsI2c.hpp"
#include <cstring>

//===========================================================================//

#ifdef SIM_MODE

PsI2c::PsI2c( uint8_t bus_index, const Logger& logger )
    : bus_index_ ( bus_index )
    , logger_    ( logger    )
{
    logger_.log_debug( "PsI2c %d: simulation mode", bus_index_ );
}

PsI2c::~PsI2c() {}

int PsI2c::write( uint8_t /*slave_addr*/, const uint8_t* /*buffer*/, uint16_t /*length*/ )
{
    return 0;
}

int PsI2c::read( uint8_t /*slave_addr*/, uint8_t* buffer, uint16_t length )
{
    std::memset( buffer, 0, length );
    return 0;
}

#else // !SIM_MODE

#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

PsI2c::PsI2c( uint8_t bus_index, const Logger& logger )
    : fd_        ( -1        )
    , bus_index_ ( bus_index )
    , logger_    ( logger    )
{
    char devpath[32];
    snprintf( devpath, sizeof(devpath), "/dev/i2c-%d", bus_index_ );

    fd_ = open( devpath, O_RDWR );
    if ( fd_ < 0 )
    {
        logger_.log_error( "PsI2c %d: failed to open %s", bus_index_, devpath );
    }
    else
    {
        logger_.log_debug( "PsI2c %d: opened %s", bus_index_, devpath );
    }
}

PsI2c::~PsI2c()
{
    if ( fd_ >= 0 )
    {
        close( fd_ );
    }
}

int PsI2c::write( uint8_t slave_addr, const uint8_t* buffer, uint16_t length )
{
    std::lock_guard<std::mutex> lock( mutex_ );

    struct i2c_msg msg;
    msg.addr  = slave_addr;
    msg.flags = 0;
    msg.len   = length;
    msg.buf   = const_cast<uint8_t*>(buffer);

    struct i2c_rdwr_ioctl_data data;
    data.msgs  = &msg;
    data.nmsgs = 1;

    if ( ioctl( fd_, I2C_RDWR, &data ) < 0 )
    {
        logger_.log_error( "PsI2c %d: write to 0x%02X failed", bus_index_, slave_addr );
        return -1;
    }

    return 0;
}

int PsI2c::read( uint8_t slave_addr, uint8_t* buffer, uint16_t length )
{
    logger_.log_debug( "PsI2c %d: reading %d bytes from 0x%02X", bus_index_, length, slave_addr );
    std::lock_guard<std::mutex> lock( mutex_ );

    struct i2c_msg msg;
    msg.addr  = slave_addr;
    msg.flags = I2C_M_RD;
    msg.len   = length;
    msg.buf   = buffer;

    struct i2c_rdwr_ioctl_data data;
    data.msgs  = &msg;
    data.nmsgs = 1;

    if ( ioctl( fd_, I2C_RDWR, &data ) < 0 )
    {
        logger_.log_error( "PsI2c %d: read from 0x%02X failed", bus_index_, slave_addr );
        return -1;
    }

    return 0;
}

#endif // SIM_MODE

