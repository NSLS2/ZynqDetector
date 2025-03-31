#pragma once

#include <map>
#include <cstdint>
#include <variant>

#include "concepts.hpp"
#include "queue.hpp"

template<typename T>
requires IsEitherType<T, PLI2C, PSI2C>
class LTC2309 : public I2CDevice<T>
{
protected:
    T&                         i2c_;
    uint8_t                    i2c_addr_;

    std::unique_ptr<AccessReq> req_;
    
    QueueHandle_t              req_queue_;
    bool                       is_single_ended_;

    std::map<int, int> chan_assign_;  // stores <variable:channel>
                                      // defined by detector and passed to the constructor

public:
    LTC2309( const T&                  i2c
           , const uint8_t             i2c_addr
           , const bool                is_single_ended
           , const std::map<int, char> chan_assign
           )
        requires IsSameType<T, PLI2C>;

    ~LTC2309() = default;

    void read( const uint8_t chan )
        requires IsSameType<T, PLI2C>;
};


#include "LTC2309.tpp"
