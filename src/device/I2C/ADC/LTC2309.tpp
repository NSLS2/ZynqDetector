#include "LTC2309.hpp"

template<typename T>
LTC2309::LTC2309( T& i2c,
                , uint8_t i2c_addr
                , QueueHandle_t req_queue
                , bool is_single_ended
                , std::map<int, char> chan_assign
                )
requires IsSameType<T, PSI2C>
                : I2CDevice        ( i2c, i2c_addr, req_queue, chan_assign )
                , is_single_ended_ ( is_single_ended ? 0x8 : 0             )
{}

//============================================
// LTC2309 read.
// Requires the ID of the physical variable.
//============================================
template<typename T>
void LTC2309::read( uint8_t var )
    requires IsSameType<T, PSI2C>
{
    char buff[2];
    if( chan_assign_.find(var) != chan_assign_.end() )
    {
        char chan = chan_assign[var];
        req->data[0] = single
                    | ((chan & 0x1)<<2)
                    | ((chan & 0x6)>>1);
        req->length = 1;
        req->read = 0;
        
        XQueueSend( req_queue_,
            	    req_,
                    0UL );

        req->data[0] = 0;
        req->data[1] = 0;

        req->length = 2;
        req->read = 1;

        XQueueSend( req_queue_,
            	    req_,
                    0UL );
    }
}
//============================================

