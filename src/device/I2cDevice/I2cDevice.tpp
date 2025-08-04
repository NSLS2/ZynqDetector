#include "FreeRTOS.h"


//============================================
// I2cDevice constructor.
//============================================
template< typename I2cType >
I2cDevice<I2cType>::I2cDevice( /*const I2cType&                     i2c
                             , */uint8_t                            i2c_addr
                             , const QueueHandle_t                req_queue
                             //, const std::map<uint16_t, uint8_t>& chan_assign
                             , const Logger&                      logger
                             )
//   requires IsSameType<T, PsI2c>
                               : /*i2c_         ( i2c                                )
                               , */i2c_addr_    ( i2c_addr                           )
                               , req_queue_   ( req_queue                          )
                               //, chan_assign_ ( chan_assign                        )
                               , logger_      ( logger                             )
{}

////============================================
//// I2cDevice write.
//// Requires the ID of the physical variable.
////============================================
//template<typename T>
//void I2cDevice<T>::write( const uint16_t var, const uint8_t data )
////requires IsSameType<T, PsI2c>
//{
//    if( chan_assign_.find(var) != chan_assign_.end() )
//    {
//        pack_req();
//        
//        xQueueSend( req_queue_,
//            	    req_,
//                    0UL );
//    }
//    //else
//    //{
//    //    log_error( "Variable %u not found\n", static_cast<uint32_t>(variable) );
//    //}
//}
////============================================
//
////============================================
//// I2cDevice read.
//// Requires the ID of the physical variable.
////============================================
//template<typename T>
//void I2cDevice<T>::read( uint16_t var )
////requires IsSameType<T, PsI2c>
//{
//    if( chan_assign_.find(var) != chan_assign_.end() )
//    {
//        pack_req( chan );
//        
//        xQueueSend( req_queue_,
//            	    req_,
//                    0UL );
//    }
//    //else
//    //{
//    //    log_error( "Variable %u not found\n", static_cast<uint32_t>(variable) );
//    //}
//}
////============================================
