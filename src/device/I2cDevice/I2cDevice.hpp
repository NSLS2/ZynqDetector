#pragma once

#include <map>
#include <cstdint>
#include <variant>
#include <memory>

#include "concepts.hpp"
#include "queue.hpp"
#include "Logger.hpp"

//class PlI2c;

template< typename I2cType >
class I2cDevice
{

    using AccessReqType = typename AccessReqTypeSelector<I2cType>::type;

protected:
//    I2cType&                     i2c_;
    uint8_t                      i2c_addr_;
    
    AccessReqType                req_;
    QueueHandle_t                req_queue_;

    //std::map<uint16_t, uint8_t>  chan_assign_;  // stores <variable:channel>
                                                // defined by detector and passed to the constructor

    const Logger& logger_;

public:

    //struct I2CInfo
    //{
    //    std::shared_ptr<I2cType> i2c;
    //    uint8_t                  chan;
    //};

    I2cDevice( /*const I2cType&                     i2c
             , */uint8_t                            i2c_addr
             , const QueueHandle_t                req_queue
             //, const std::map<uint16_t, uint8_t>& chan_assign
             , const Logger&                      logger
             );
    //requires IsSameType<T, PlI2c>;

    ~I2cDevice() = default;

};

#include "I2cDevice.tpp"
