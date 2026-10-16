/**
 * @file GermaniumDetector.hpp
 * @brief Class template definition of `Tmp100`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

#pragma once

//===========================================================================//

#include <concepts>

#include "FreeRTOS.h"

#include "I2cDevice.hpp"

//===========================================================================//

/**
 * @brief Class template Tmp100.
 */
template <typename I2cType>
//requires IsEitherType<I2cType, PlI2c, PsI2c>
class Tmp100 : public I2cDevice<I2cType>
{
    using I2cDevice<I2cType>::i2c_addr_;
    using I2cDevice<I2cType>::req_;
    using I2cDevice<I2cType>::req_queue_;

public:

    Tmp100( /*const I2cType&                     i2c
          , */const uint8_t                      i2c_addr
          , const QueueHandle_t                req_queue
          , const Logger&                      logger
          );
//        requires IsSameType<T, PsI2c>;

    ~Tmp100() = default;

    /**
     * @brief Read Tmp100.
     */
    void read( uint16_t op );
//        requires IsSameType<T, PsI2c>;

};

//===========================================================================//

#include "Tmp100.tpp"
