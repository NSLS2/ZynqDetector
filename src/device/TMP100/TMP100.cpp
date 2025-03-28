#include "concepts.hpp"
#include "TMP100.hpp"


//============================================
// TMP100 constructor.
//============================================
template <typename T_I2C, typename T_I2C_REQ>
TMP100<T_I2C, I_I2C_REQ>::TMP100( T_I2C&        i2c
                                , uint8_t       i2c_addr
                                , QueueHandle_t req_queue
                                )
    requires IsSameType<T_I2C, PLI2C>
                                : i2c_       ( i2c       )
                                , i2c_addr_  ( i2c_addr  )
                                , req_queue_ ( req_queue )
{}
//============================================
template <typename T_I2C, typename T_I2C_REQ>
TMP100<T_I2C, I_I2C_REQ>::TMP100( T_I2C&        i2c
                                , uint8_t       i2c_addr
                                , QueueHandle_t req_queue
                                )
    requires IsSameType<T_I2C, PSI2C>
                                : i2c_       ( i2c       )
                                , i2c_addr_  ( i2c_addr  )
                                , req_queue_ ( req_queue )
{}
//============================================


//============================================
// TMP100 read.
// Requires the ID of the physical variable.
//============================================
TMP100<T_I2C, I_I2C_REQ>::read( uint8_t chan )
    requires IsSameType<T_I2C, PLI2C>
{
    req_.data[0] = 0x10 + chan_assign[var];
    req_.read = 1;
        
    // send req to queue
}
//============================================
TMP100<T_I2C, I_I2C_REQ>::read( uint8_t chan )
requires IsSameType<T_I2C, PSI2C>
{
req_.data[0] = 0x10 + chan_assign[var];
req_.read = 1;
    
// send req to queue
}
//============================================
