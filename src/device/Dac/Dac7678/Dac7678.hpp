/**
 * @file Dac7678.hpp
 * @brief Class template definition of `Dac7678`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <cstdint>
#include <map>

#include "concepts.hpp"
#include "queue.hpp"
#include "Logger.hpp"

//===========================================================================//

template< typename I2cType >
//requires IsEitherType<T, PlI2c, PsI2c>
class Dac7678 : public I2cDevice<I2cType>
{
    using I2cDevice<I2cType>::i2c_addr_;
    using I2cDevice<I2cType>::req_;
    using I2cDevice<I2cType>::req_queue_;
    using I2cDevice<I2cType>::logger_;

private:

    const uint16_t DAC_INT_REF = 0;

public:

    Dac7678( uint8_t              i2c_addr
           , const QueueHandle_t  req_queue
           , const Logger&        logger
           );
//        requires IsSameType<T, PlI2c>;

    ~Dac7678() = default;

    /**
     * @brief Write to DAC7678.
     */
    void write( uint16_t data, uint8_t chan );

    /**
     * @brief Read from DAC7678.
     */
    void read( uint16_t op, uint8_t chan );
};

//===========================================================================//

#include "Dac7678.tpp"
