#pragma once

#include <concepts>

#include "FreeRTOS.h"

template <typename T_I2C, typename T_I2C_REQ>
class TMP100
{
private:
    T_I2C&  i2c_;
    uint8_t i2c_addr_;
    T_I2C_REQ req_;
    QueueHandle_t, req_queue_;

public:
    TMP100( T_I2C i2c, uint8_t i2c_addr )
        requires IsSameType<T, PLI2C>;

    TMP100( T_I2C i2c, uint8_t i2c_addr )
        requires IsSameType<T, PSI2C>;

    ~TMP100() = default;

    void read( uint8_t chan, uint16_t data )
        requires IsSameType<T, PSI2C>;
    void read( uint8_t chan, uint16_t data )
        requires IsSameType<T, PLI2C>;
};

#include "TMP100.cpp"