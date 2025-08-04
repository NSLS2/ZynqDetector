#pragma once

#include <concepts>

#include "FreeRTOS.h"

#include "I2cDevice.hpp"

template <typename I2cType>
//requires IsEitherType<I2cType, PlI2c, PsI2c>
class Tmp100 : public I2cDevice<I2cType>
{

    //using I2cDevice<I2cType>::i2c_;
    using I2cDevice<I2cType>::i2c_addr_;
    using I2cDevice<I2cType>::req_;
    using I2cDevice<I2cType>::req_queue_;
    //using I2cDevice<I2cType>::chan_assign_;

public:

    Tmp100( /*const I2cType&                     i2c
          , */const uint8_t                      i2c_addr
          , const QueueHandle_t                req_queue
          //, const std::map<uint16_t, uint8_t>& chan_assign
          , const Logger&                      logger
          );
//        requires IsSameType<T, PsI2c>;

    ~Tmp100() = default;

    void read( uint16_t op );
//        requires IsSameType<T, PsI2c>;

    //void read( uint8_t chan, uint16_t data )
    //    requires IsSameType<T, PlI2c>;
};

#include "Tmp100.tpp"
