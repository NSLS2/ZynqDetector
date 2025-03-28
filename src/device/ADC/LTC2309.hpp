#pragma once

#include <map>
#include <cstdint>
#include <variant>

#include "concepts.hpp"

template<typename T_I2C>
requires IsEitherType<T_I2C, PLI2C, PSI2C>
class LTC2309
{
private:
    T_I2C&    i2c_;
    uint8_t   i2c_addr_;
    bool      is_single_ended_;
    std::variant<PLI2CReq, PSI2CReq> req_;

    std::map<int, int> chan_assign_;  // stores <variable:channel>
                                      // defined by detector and passed to the constructor

public:
    LTC2309( T_I2C&   i2c
           , uint8_t i2c_addr
           , bool is_single_ended
           , std::map<int, char> chan_assign
           )
        requires IsSameType<T, PLI2C>;

    ~LTC2309() = default;

    void read( uint8_t chan )
        requires IsSameType<T, PLI2C>;
};
