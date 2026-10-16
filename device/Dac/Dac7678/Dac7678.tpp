/**
 * @file Dac7678.tpp
 * @brief Member function definitions of `Dac7678`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include <cstdint>

#include "FreeRTOS.h"

//===========================================================================//

/**
 * @brief Dac7678 constructor.
 * @param i2c_addr I2C address of DAC7678.
 * @param req_queue Queue for passing DAC7678 access requests.
 * @param logger Reference to the logger.
 */
template< typename I2cType >
Dac7678<I2cType>::Dac7678( uint8_t              i2c_addr
                         , const QueueHandle_t  req_queue
                         , const Logger&        logger
                         )
//requires IsSameType<T, PsI2c>
                           : I2cDevice<I2cType> ( i2c_addr, req_queue, logger )
{
    // enable internal reference
    req_.op      = DAC_INT_REF;
    req_.addr    = i2c_addr_;
    req_.read    = 0;
    req_.length  = 3;
    req_.data[0] = 0x80;
    req_.data[1] = 0x00;
    req_.data[2] = 0x10;
    
    xQueueSend( req_queue_, &req_, 0UL );
}

//===========================================================================//

/**
 * @brief Dac7678 write.
 * @param chan Channel to be written.
 * @param data Data to be written to the channel.
 */
template< typename I2cType >
void Dac7678<I2cType>::write( uint16_t data, uint8_t chan )
//    requires IsSameType<T, PsI2c>
{
    req_.op      = WRITE;
    req_.addr    = i2c_addr_;
    req_.read    = 0;
    req_.length  = 3;
    req_.data[0] = 0x30 + chan;
    req_.data[1] = static_cast<uint8_t>(data >> 4);
    req_.data[2] = static_cast<uint8_t>((data && 0x0F) << 4);
    
    xQueueSend( req_queue_, &req_, 0UL );
}

//===========================================================================//

/**
 * @brief Dac7678 read.
 * @param op Operation ID. Used in response message.
 * @param chan Channel to be read
 */
template< typename I2cType >
void Dac7678<I2cType>::read( uint16_t op, uint8_t chan )
//    requires IsSameType<T, PsI2c>
{
    req_.op      = op;
    req_.addr    = i2c_addr_;
    req_.data[0] = 0x10 + chan;
    req_.length  = 1;
    req_.read    = 1;
    
    xQueueSend( req_queue_, &req_, 0UL );
}

//===========================================================================//
