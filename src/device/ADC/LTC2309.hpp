#pragma once

#include <map>
#include <cstidint>

template<typename T_I2C, typename T_I2C_REQ>
class LTC2309
{
private:
    T_I2C&    i2c_;
    uint8_t   i2c_addr_;
    bool      is_single_ended_;
    T_I2C_REQ req_;

    std::map<int, int> chan_assign_;  // stores <variable:channel>
                                      // defined by detector and passed to the constructor

public:
    LTC2309( T_I2C&   i2c
           , uint8_t i2c_addr
           , bool is_single_ended
           , std::map<int, char> chan_assign
           );

    ~LTC2309() = default;

    void read( uint8_t chan );
};
