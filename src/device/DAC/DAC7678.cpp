#include "FreeRTOS.h"

#include "DAC7678.hpp"



//============================================
// DAC7678 constructor.
//============================================
template <typename T_I2C>
DAC7678<T_I2C>::DAC7678( const T_I2C&                       i2c
                       , const uint8_t                      i2c_addr
                       , const QueueHandle_t                req_queue
                       , const std::map<uint16_t, uint8_t>& chan_assign
                       )
    requires IsSameType<T_I2C, PSI2C>
                       : i2c_        ( i2c                                )
                       , i2c_addr_   ( i2c_addr                           )
                       , req_        ( std::make_unique<PSI2CAccessReq>() )
                       , req_queue_  ( req_queue                          )
                       , chan_assign_( chan_assign                        )
{
    // enable internal reference
    req_->op      = DAC_INT_REF;
    req_->length  = 3;
    req_->addr    = i2c_addr_;
    req_->data[0] = 0x80;
    req_->data[1] = 0x00;
    req_->data[2] = 0x10;
    req_->read    = 0;
    
    XQueueSend( req_queue_,
                req,
                0UL );
}

//============================================
// DAC7678 write.
// Requires the ID of the physical variable.
//============================================
void DAC7678<typename T>::write( const uint16_t variable, const uint8_t data )
    requires IsSameType<T, PSI2C>
{
    if( chan_assign_.find(variable) != chan_assign_.end() )
    {
        req_->op      = op;
        req_->length  = 3;
        req_->addr    = i2c_addr_;
        req_->read    = 0;
        req_->data[0] = 0x30 + chan_assign[variable];
        req_->data[1] = static_cast<uint8_t>(data >> 4);
        req_->data[2] = static_cast<uint8_t>((data && 0x0F) << 4);
        
        XQueueSend( req_queue_,
            	    req,
                    0UL );
    }
    else
    {
        log_error( "Variable %u not found\n", static_cast<uint32_t>(variable) );
    }
}
//============================================

//============================================
// DAC7678 read.
// Requires the ID of the physical variable.
//============================================
void DAC7678<typename T>::read( uint8_t chan )
    requires IsSameType<T, PSI2C>
{
    if( chan_assign_.find(var) != chan_assign_.end() )
    {
        req_.data[0] = 0x10 + chan_assign[var];
        req_.read = 1;
        
        // send req to queue
    }
}
//============================================
