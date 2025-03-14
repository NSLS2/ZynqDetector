#include "TMP100.hpp"


//============================================
// TMP100 constructor.
//============================================
template <typename T_I2C, typename T_I2C_REQ>
TMP100<T_I2C, I_I2C_REQ>::TMP100( T_I2C&        i2c
                                , uint8_t       i2c_addr
                                )
    : i2c_      ( i2c      )
    , i2c_addr_ ( i2c_addr )
{}
//============================================


//============================================
// DAC7678 read.
// Requires the ID of the physical variable.
//============================================
TMP100<T_I2C, I_I2C_REQ>::read( uint8_t chan )
{
    req_.data[0] = 0x10 + chan_assign[var];
    req_.read = 1;
        
    // send req to queue
}
//============================================
