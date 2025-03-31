#include "FreeRTOS.h"

#include "DAC7678.hpp"


//============================================
// DAC7678 constructor.
//============================================
template <typename T>
DAC7678<T>::DAC7678( const TC&                          i2c
                   , const uint8_t                      i2c_addr
                   , const QueueHandle_t                req_queue
                   , const std::map<uint16_t, uint8_t>& chan_assign
                   )
requires IsSameType<T, PSI2C>
                   : Base ( i2c, i2c_addr, req_queue, chan_assign )
{
    // enable internal reference
    req_->op      = DAC_INT_REF;
    req_->addr    = i2c_addr_;
    req_->read    = 0;
    req_->length  = 3;
    req_->data[0] = 0x80;
    req_->data[1] = 0x00;
    req_->data[2] = 0x10;
    
    XQueueSend( req_queue_, req, 0UL );
}

//============================================
// DAC7678 write.
// Requires the ID of the physical variable.
//============================================
template<typename T>
void DAC7678<T>::write( const uint16_t variable, const uint8_t data )
    requires IsSameType<T, PSI2C>
{
    if( chan_assign_.find(variable) != chan_assign_.end() )
    {
        req_->op      = op;
        req_->addr    = i2c_addr_;
        req_->read    = 0;
        req_->length  = 3;
        req_->data[0] = 0x30 + chan_assign[variable];
        req_->data[1] = static_cast<uint8_t>(data >> 4);
        req_->data[2] = static_cast<uint8_t>((data && 0x0F) << 4);
        
        XQueueSend( req_queue_, req, 0UL );
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
template<typename T>
void DAC7678<T>::read( const uint8_t chan )
    requires IsSameType<T, PSI2C>
{
    if( chan_assign_.find(var) != chan_assign_.end() )
    {
        req_->op      = op;
        req_->data[0] = 0x10 + chan_assign[var];
        req_->length  = 1;
        req_->read    = 1;
        
        XQueueSend( req_queue_, req, 0UL );
    }
}
//============================================

#include "DAC7678.tpp"
