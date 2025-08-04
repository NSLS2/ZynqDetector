#pragma once

// C++ includes
#include <cstddef>
#include <iterator>
#include <portmacro.h>
#include <type_traits>
#include <memory>

// FreeRTOS includes
#include "FreeRTOS.h"
//#include "msg.hpp"
#include "task.h"
#include "queue.h"
#include "timers.h"

// Xilinx includes
#include "xil_printf.h"
#include "xparameters.h"

// Project includes
//#include "ZynqDetector.hpp"
//#include "pynq_ssd_msg.hpp"

#define TIMER_ID	1
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define TIMER_CHECK_THRESHOLD	9


//Udp_Msg_Handler::Udp_Msg_Handler()
//{
//    create_detector_instr_map();
//}

#if (configSUPPORT_STATIC_ALLOCATION == 1)
#define QUEUE_BUFFER_SIZE		100

uint8_t ucQueueStorageArea[ QUEUE_BUFFER_SIZE ];
StackType_t xStack1[ configMINIMAL_STACK_SIZE ];
StackType_t xStack2[ configMINIMAL_STACK_SIZE ];
StaticTask_t xTxBuffer,xRxBuffer;
StaticTimer_t xTimerBuffer;
static StaticQueue_t xStaticQueue;
#endif

//===============================================================
// Constructor.
//===============================================================
template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
ZynqDetector< DerivedDetector
            , DerivedNetwork
            , DerivedZynq
            >::ZynqDetector()
{}

template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::set_zynq(std::unique_ptr<DerivedZynq> z)
{
    zynq_ = std::move(z);
}

template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::set_network(std::unique_ptr<DerivedNetwork> n)
{
    network_ = std::move(n);
}


/*
//===============================================================

//===============================================================
//===============================================================
ZynqDetector::ZynqDetector( void )
{
	const TickType_t x1second = pdMS_TO_TICKS( DELAY_1_SECOND );

    // Network
    net = std::make_unique( Network::createInstance() );

    if ( net == nullptr )
    {
        report_error( "Network initialization failed with error code ", err_code, uint32_t fail_num);        
    }

    queue_init();

    create_tasks();

	// Create a timer
	xPollTimer = xTimerCreate( (const char *) "Timer",
							   x1second,
							   pdTRUE,
							   (void *) TIMER_ID,
							   vTimerCallback);
	configASSERT( xPollTimer );

	xTimerStart( xPollTimer, 0 );


	// Start the tasks
	vTaskStartScheduler();

	for( ;; );
}
//===============================================================
*/


//===============================================================
//  Single register access.
//  Assembles fast access request and sends it to the queue.
//===============================================================
//template< typename DerivedDetector
//        , typename DerivedNetwork
//        , typename DerivedZynq
//        >
//void ZynqDetector< DerivedDetector
//                 , DerivedNetwork
//                 , DerivedZynq
//                 >::register_single_access_request_process( udp_rx_msg_t& msg )
//{
//    fast_access_req_t req;
//    req.op   = msg->op;
//    req.data = msg->data;
//    xQueueSend( single_register_access_request_queue,
//            	req,
//                0UL );
//}
//===============================================================



//===============================================================
// Write failure number to register.
//===============================================================
//void ZynqDetector:set_fail_num( uint32_t failure_num )
//{
//    reg_wr( REG_DEVICE_STATUS, failure_num );
//}




//===============================================================
//===============================================================
//template< typename DerivedDetector
//        , typename DerivedNetwork
//        , typename DerivedZynq
//        >
//void ZynqDetector< DerivedDetector
//                 , DerivedNetwork
//                 , DerivedZynq
//                 >::network_init( std::unique_ptr<DerivedNetwork> network )
//{
//    network_ = std::move( network );
//}
//===============================================================



//===============================================================
//===============================================================
template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::task_init()
{
    create_network_tasks();
    create_device_access_tasks();
    //create_polling_tasks();
}
//===============================================================


//===============================================================
//===============================================================
template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::create_network_tasks()
{
    network_->base_->create_network_tasks();
}
//===============================================================


//===============================================================
//===============================================================
template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::create_device_access_tasks()
{
    zynq_->base_->create_device_access_tasks();
}
//===============================================================
