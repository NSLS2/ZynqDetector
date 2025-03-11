#include "DAC7678.hpp"



//============================================
// DAC7678 constructor.
//============================================
DAC7678::DAC7678( uint8_t i2c_addr, psi2c_req_queue, std::map<int, char> chan_assign)
                : i2c_addr_(i2c_addr)
                , req_queue_(req_queue)
                , chan_assign_(chan_assign)
{
    // enable internal reference
    psi2c_req_.addr = i2c_addr_;
    psi2c_req_.data[0] = 0x80;
    psi2c_req_.data[1] = 0x00;
    psi2c_req_.data[2] = 0x10;

    // send req to queue
}

//============================================
// DAC7678 write.
// Requires the ID of the physical variable.
//============================================
DAC7678::write( uint8_t variable, uint16_t data )
{
    if( chan_assign_.find(var) != chan_assign_.end() )
    {
        psi2c_req_.data[0] = 0x30 + chan_assign[var];
        psi2c_req_.data[1] = static_cast<uint8_t>(data >> 4);
        psi2c_req_.data[2] = static_cast<uint8_t>((data && 0x0F) << 4);
        psi2c_req_.length = 3;
        psi2c_req_.read = 0;
                
        // send req to queue
    }
    else
    {
        // error
    }
}
//============================================

//============================================
// DAC7678 read.
// Requires the ID of the physical variable.
//============================================
DAC7678::read( uint8_t chan )
{
    if( chan_assign_.find(var) != chan_assign_.end() )
    {
        psi2c_req_.data[0] = 0x10 + chan_assign[var];
        psi2c_req_.read = 1;
        
        // send req to queue
    }
}
//============================================
