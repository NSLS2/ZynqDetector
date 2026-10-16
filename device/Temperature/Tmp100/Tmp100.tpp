/**
 * @file Tmp100.tpp
 * @brief Member function definitions of `Tmp100`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "concepts.hpp"

//===========================================================================//
/*
 * @brief Tmp100 constructor.
 * @param i2c_addr I2C address of Tmp100.
 * @param req_queue Queue for passing Tmp100 access requests.
 * @param logger Reference of the logger.
 */
template <typename I2cType>
Tmp100<I2cType>::Tmp100( const uint8_t         i2c_addr
                       , const QueueHandle_t   req_queue
                       , const Logger&         logger
                       )
    //requires IsSameType<T_I2C, PsI2c>
                     : I2cDevice<I2cType> ( i2c_addr, req_queue, logger )
{}

//===========================================================================//

/*
 * @brief Tmp100 read.
 * @param op Operation code. Used in response message.
 */
template <typename I2cType>
void Tmp100<I2cType>::read( uint16_t op )
//    requires IsSameType<T_I2C, PsI2c>
{
    req_.op      = op;
    req_.addr    = i2c_addr_;
    req_.read    = 1;
    req_.length  = 1;
    req_.data[0] = 0x10;
    
    xQueueSend( req_queue_, &req_, 0UL );
}
//
//===========================================================================//
