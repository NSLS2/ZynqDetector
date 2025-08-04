#pragma once

#include <cstdint>
#include <map>

#include "concepts.hpp"
#include "queue.hpp"
#include "Logger.hpp"

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

    Dac7678( uint8_t                             i2c_addr
           , const QueueHandle_t                 req_queue
           , const Logger&                       logger
           );
//        requires IsSameType<T, PlI2c>;

    ~Dac7678() = default;

    void write( uint16_t data, uint8_t chan );

    void read( uint16_t op, uint8_t chan );
//        requires IsSameType<T, PlI2c>;
};


#include "Dac7678.tpp"
