/**
 * @file Ltc2309.hpp
 * @brief Class template definition of `Ltc2309`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <map>
#include <cstdint>
#include <variant>

#include "concepts.hpp"
#include "queue.hpp"
#include "Logger.hpp"

#include "I2cDevice.hpp"
#include "PsI2c.hpp"

//===========================================================================//

template<typename I2cType>
//requires IsEitherType<I2cType, PlI2c, PsI2c>
class Ltc2309 : public I2cDevice<I2cType>
{
    using I2cDevice<I2cType>::i2c_addr_;
    using I2cDevice<I2cType>::req_;
    using I2cDevice<I2cType>::req_queue_;

protected:
    uint8_t   is_single_ended_;

public:
    Ltc2309( uint8_t               i2c_addr
           , const QueueHandle_t   req_queue
           , bool                  is_single_ended
           , const Logger&         logger
           );
//        requires IsEitherType<T, PlI2c, PsI2c> && IsSameType<T, PsI2c>;

    ~Ltc2309() = default;

    /**
     * @brief Read LTC2309.
     */
    void read( uint16_t op, uint8_t chan );
};

//===========================================================================//

#include "Ltc2309.tpp"
