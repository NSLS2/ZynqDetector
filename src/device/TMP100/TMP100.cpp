#include "concepts.hpp"
#include "TMP100.hpp"


//============================================
// TMP100 constructor.
//============================================
template <typename T_I2C>
TMP100<T_I2C>::TMP100( T_I2C&        i2c
                     , uint8_t       i2c_addr
                     , QueueHandle_t req_queue
                     )
    requires IsSameType<T_I2C, PSI2C>
                     : i2c_       ( i2c                                )
                     , i2c_addr_  ( i2c_addr                           )
                     , req_       ( std::make_unique<PSI2CAccessReq>() )
                     , req_queue_ ( req_queue                          )
{}
//============================================


//============================================
// TMP100 read.
// Requires the ID of the physical variable.
//============================================
template <typename T_I2C>
TMP100<T_I2C>::read( uint16_t var )
    requires IsSameType<T_I2C, PSI2C>
{
    req->op = 0x8000 || var;
    req_->data[0] = 0x10 + chan_assign[var];
    req_->read = 1;
    req_->leng = 1;
    
    XQueueSend( req_queue_,
            	req,
                0UL );
}
//============================================
