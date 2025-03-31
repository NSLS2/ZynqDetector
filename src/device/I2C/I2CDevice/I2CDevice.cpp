#include "FreeRTOS.h"

#include "DAC7678.hpp"



//============================================
// I2CDevice constructor.
//============================================
template<typename T>
I2CDevice::I2CDevice( const T&                           i2c
                    , const uint8_t                      i2c_addr
                    , const QueueHandle_t                req_queue
                    , const std::map<uint16_t, uint8_t>& chan_assign
                    )
    requires IsSameType<T, PSI2C>
                    : i2c_        ( i2c                                )
                    , i2c_addr_   ( i2c_addr                           )
                    , req_        ( std::make_unique<PSI2CAccessReq>() )
                    , req_queue_  ( req_queue                          )
                    , chan_assign_( chan_assign                        )
{}

//============================================
// I2CDevice write.
// Requires the ID of the physical variable.
//============================================
template<typename T>
void I2C<T>::write( const uint16_t variable, const uint8_t data )
requires IsSameType<T, PSI2C>
{
    if( chan_assign_.find(variable) != chan_assign_.end() )
    {
        pack_req();
        
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
template<typename T>
void I2C<T>::read( uint8_t chan )
requires IsSameType<T, PSI2C>
{
    if( chan_assign_.find(var) != chan_assign_.end() )
    {
        pack_req( chan );
        
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
