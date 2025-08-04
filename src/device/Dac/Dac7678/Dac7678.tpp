#include <cstdint>
#include <map>

#include "FreeRTOS.h"


//============================================
// Dac7678 constructor.
//============================================
template< typename I2cType >
Dac7678<I2cType>::Dac7678( uint8_t                            i2c_addr
                         , const QueueHandle_t                req_queue
                         , const Logger&                      logger
                         )
//requires IsSameType<T, PsI2c>
                           : I2cDevice<I2cType> ( i2c_addr, req_queue, logger )
{
    // enable internal reference
    req_.op      = DAC_INT_REF;
    req_.addr    = i2c_addr_;
    req_.read    = 0;
    req_.length  = 3;
    req_.data[0] = 0x80;
    req_.data[1] = 0x00;
    req_.data[2] = 0x10;
    
    xQueueSend( req_queue_, &req_, 0UL );
}

//============================================
// Dac7678 write.
// Requires the ID of the physical variable.
//============================================
template< typename I2cType >
void Dac7678<I2cType>::write( uint16_t data, uint8_t chan )
//    requires IsSameType<T, PsI2c>
{
    req_.op      = WRITE;
    req_.addr    = i2c_addr_;
    req_.read    = 0;
    req_.length  = 3;
    req_.data[0] = 0x30 + chan;
    req_.data[1] = static_cast<uint8_t>(data >> 4);
    req_.data[2] = static_cast<uint8_t>((data && 0x0F) << 4);
    
    xQueueSend( req_queue_, &req_, 0UL );
}
//============================================

//============================================
// Dac7678 read.
// Requires the ID of the physical variable.
//============================================
template< typename I2cType >
void Dac7678<I2cType>::read( uint16_t op, uint8_t chan )
//    requires IsSameType<T, PsI2c>
{
    req_.op      = op;
    req_.addr    = i2c_addr_;
    req_.data[0] = 0x10 + chan;
    req_.length  = 1;
    req_.read    = 1;
    
    xQueueSend( req_queue_, &req_, 0UL );
}
//============================================

