/**
 * @file Ltc2309.cpp
 * @brief Member function definitions of `Ltc2309`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "FreeRTOS.h"
#include "queue.h"

//===========================================================================//

/**
 * @brief Ltc2309 constructor.
 * @param i2c_addr I2C address of LTC2309.
 * @param req_queu Queue for pass LTC2309 access requests.
 * @param is_single_ended Whether it's configured as single ended.
 * @param logger Reference to the logger.
 */
template< typename I2cType >
//requires IsEitherType<T, PlI2c, PsI2c> && IsSameType<T, PsI2c>
Ltc2309<I2cType>::Ltc2309
    ( uint8_t               i2c_addr
    , const QueueHandle_t   req_queue
    , bool                  is_single_ended
    , const Logger&         logger
    )
    : I2cDevice<I2cType> ( i2c_addr, req_queue, logger )
    , is_single_ended_   ( is_single_ended ? 0x8 : 0 )
{}

//===========================================================================//

/**
 * @brief Ltc2309 read.
 * @param op Operation code. Used in response message.
 * @param chan Channel to be read.
 */
template< typename I2cType >
//requires IsSameType<T, PsI2c>
void Ltc2309<I2cType>::read( uint16_t op, uint8_t chan )
{
    req_.op      = op;
    req_.addr    = i2c_addr_;
    req_.data[0] = is_single_ended_
                 | ((chan & 0x1)<<2)
                 | ((chan & 0x6)>>1);
    req_.length  = 1;
    req_.read    = 0;
    
    xQueueSend( req_queue_,
        	    &req_,
                0UL );

    req_.data[0] = 0;
    req_.data[1] = 0;

    req_.length  = 2;
    req_.read    = 1;

    xQueueSend( req_queue_,
        	    &req_,
                0UL );
    
}
//===========================================================================//

