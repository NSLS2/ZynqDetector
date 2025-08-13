/**
 * @file I2cDevice.cpp
 * @brief Member function definitions of `I2cDevice`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "FreeRTOS.h"

//===========================================================================//

/**
 * @brief I2cDevice constructor.
 */
template< typename I2cType >
I2cDevice<I2cType>::I2cDevice( uint8_t              i2c_addr
                             , const QueueHandle_t  req_queue
                             , const Logger&        logger
                             )
//   requires IsSameType<T, PsI2c>
                               : i2c_addr_    ( i2c_addr  )
                               , req_queue_   ( req_queue )
                               , logger_      ( logger    )
{}

//===========================================================================//
