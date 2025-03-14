#include "LTC2309.hpp"

LTC2309<T_I2C, T_I2C_REQ>：：LTC2309( T_I2C& i2c,
                                   , uint8_t i2c_addr
                                   , bool is_single_ended
                                   , std::map<int, char> chan_assign
                                   )
    : i2c_        ( i2c                       )
    , i2c_addr_   ( i2c_addr                  )
    , signle      ( is_single_ended ? 0x8 : 0 )
    , chan_assign_( chan_assign               )
{}

//============================================
// LTC2309 read.
// Requires the ID of the physical variable.
//============================================
void LTC2309::read( uint8_t chan )
{
    char buff[2];
    if( chan_assign_.find(var) != chan_assign_.end() )
    {
        char chan = chan_assign[var];
        req.data[0] = single | ((chan & 0x1)<<2) | ((chan & 0x6)>>1);
        req.length = 1;
        req.read = 0;
        
        // send req to queue

        req.data[0] = 0;
        req.data[1] = 0;

        req.length = 2;
        req.read = 1;

        // send req to queue

    }
}
//============================================
