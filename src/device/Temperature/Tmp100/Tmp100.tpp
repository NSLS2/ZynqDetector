#include "concepts.hpp"


//============================================
// Tmp100 constructor.
//============================================
template <typename I2cType>
Tmp100<I2cType>::Tmp100( const uint8_t         i2c_addr
                       , const QueueHandle_t   req_queue
                       , const Logger&         logger
                       )
    //requires IsSameType<T_I2C, PsI2c>
                     : I2cDevice<I2cType> ( i2c_addr, req_queue, logger )
{}
//============================================


//============================================
// Tmp100 read.
// Requires the ID of the physical variable.
//============================================
template <typename I2cType>
void Tmp100<I2cType>::read( uint16_t op )
//    requires IsSameType<T_I2C, PsI2c>
{
    req_.op      = op;
    req_.addr    = i2c_addr_;
    req_.read    = 1;
    req_.length  = 1;
    req_.data[0] = 0x10;
    
    xQueueSend( req_queue_, &req_, 0UL );
}
//============================================
