#pragma once

template <typename T_I2C, typename T_I2C_REQ>
class DAC7678
{
private:
    T_I2C&  i2c_;
    uint8_t i2c_addr_;
    QueueHandle_t* req_queue_;

    T_I2C_REQ req_;
    std::map<int, int> chan_assign_;  // stores <variable:channel>
                                      // defined by detector and passed to the constructor

public:
    DAC7678( T_I2C i2c, uint8_t i2c_addr, std::map<int, char> chan_assign);

    ~DAC7678() = default;

    void write( uint8_t variable, uint16_t data );

    void read( uint8_t chan, uint16_t data );
};


#include "DAC7678.cpp"
