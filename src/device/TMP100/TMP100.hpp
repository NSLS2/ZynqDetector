#pragma once

#include "FreeRTOS.h"

template <typename T_I2C, typename T_I2C_REQ>
class TMP100
{
private:
    T_I2C&  i2c_;
    uint8_t i2c_addr_;
    T_I2C_REQ req_;

public:
    TMP100( T_I2C i2c, uint8_t i2c_addr );

    ~TMP100() = default;

    void read( uint8_t chan, uint16_t data );
};

#include "TMP100.cpp"