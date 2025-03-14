#include "FreeRTOS.h"

#include "DAC7678.hpp"



//============================================
// DAC7678 constructor.
//============================================
template <typename T_I2C, typename T_I2C_REQ>
DAC7678<T_I2C, T_I2C_REQ>::DAC7678( T_I2C&        i2c
                                  , uint8_t       i2c_addr
                                  , QueueHandle_t req_queue
                                  , std::map<int, char> chan_assign
                                  )
    : i2c_        ( i2c         )
    , i2c_addr_   ( i2c_addr    )
    , req_queue_  ( req_queue   )
    , chan_assign_( chan_assign )
{
    // enable internal reference
    req_.addr = i2c_addr_;
    req_.data[0] = 0x80;
    req_.data[1] = 0x00;
    req_.data[2] = 0x10;

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
        req_.data[0] = 0x30 + chan_assign[var];
        req_.data[1] = static_cast<uint8_t>(data >> 4);
        req_.data[2] = static_cast<uint8_t>((data && 0x0F) << 4);
        req_.length = 3;
        req_.read = 0;
                
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
        req_.data[0] = 0x10 + chan_assign[var];
        req_.read = 1;
        
        // send req to queue
    }
}
//============================================
