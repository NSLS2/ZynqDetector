/**
 * @file I2cDevice.hpp
 * @brief Class definition of `I2cDevice`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include "concepts.hpp"
#include "queue.hpp"
#include "Logger.hpp"

//===========================================================================//

template< typename I2cType >
class I2cDevice
{

    using AccessReqType = typename AccessReqTypeSelector<I2cType>::type;

protected:
    uint8_t                      i2c_addr_;
    
    AccessReqType                req_;
    QueueHandle_t                req_queue_;

    const Logger& logger_;

public:

    I2cDevice( uint8_t              i2c_addr
             , const QueueHandle_t  req_queue
             , const Logger&        logger
             );
    //requires IsSameType<T, PlI2c>;

    ~I2cDevice() = default;

};

//===========================================================================//

#include "I2cDevice.tpp"
