#pragma once

class DAC7678
{
private:
    uint8_t i2c_addr_;
    req_queue_;

    PSI2CReq psi2c_req_;
    std::map<int, int> chan_assign_;  // stores <variable:channel>
                                      // defined by detector and passed to the constructor

public:
    DAC7678( uint8_t i2c_addr, psi2c_req_queue, std::map<int, char> chan_assign);

    ~DAC7678() = default;

    void write( uint8_t variable, uint16_t data );

    void read( uint8_t chan, uint16_t data );
};
