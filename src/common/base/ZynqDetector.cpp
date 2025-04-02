// C++ includes
#include <cstddef>
#include <iterator>
#include <portmacro.h>
#include <type_traits>
#include <memory>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "msg.hpp"
#include "task.h"
#include "queue.h"
#include "timers.h"

// Xilinx includes
#include "xil_printf.h"
#include "xparameters.h"

// Project includes
#include "ZynqDetector.hpp"
#include "pynq_ssd_msg.hpp"

#define TIMER_ID	1
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define TIMER_CHECK_THRESHOLD	9


Udp_Msg_Handler::Udp_Msg_Handler()
{
    create_detector_instr_map();
}

/* The queue used by the Tx and Rx tasks, as described at the top of this
file. */
static TimerHandle_t xPollTimer = NULL;
char HWstring[15] = "Hello World";
long RxtaskCntr = 0;

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
ZynqDetector::ZynqDetector( uint32_t base_addr
                          , std::unique_ptr<Network> net )
    : zynq_    ( base_addr      )
    , network_ ( std::move(net) )
{}
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


void ZynqDetector::must_override()
{
    log_error("For reference only. Implement it for the specific detector.\n" );
    exit();
}



//===============================================================
//  Single register access.
//  Assembles fast access request and sends it to the queue.
//===============================================================
void ZynqDetector::register_single_access_request_process( udp_rx_msg_t& msg )
{
    fast_access_req_t req;
    req.op   = msg->op;
    req.data = msg->data;
    xQueueSend( single_register_access_request_queue,
            	req,
                0UL );
}
//===============================================================


//===============================================================
// This task performs single register read/write operation.
//===============================================================
void ZynqDetector::reg_access_task( void *pvParameters )
{
    reg_access_req_t  req;
    reg_access_resp_t resp;

    auto param = static_cast<reg_access_task_param_t*>(pvParameters);

    while(1)
    {
        xQueueReceive( 	static_cast<QueueHandle_t*>(pvParameters),
						&req,
						portMAX_DELAY );
        
        if ( req.read )
        {
            resp.op = req.op;
            resp.data = reg_rd ( req );
            xQueueSend( (QueueHandle_t*)resp,
                        )
        }
        else
        {
            reg_wr( req.addr, req.data );
        }
    }
}
//===============================================================

/*
//===============================================================
// Wraps a task function for resource access.
//===============================================================
static void ZynqDetector::task_wrapper(void* param, void (Derived::*task)())
{
    auto obj = statid_cast<Derived*>(param);
    if( obj )
    {
        obj->*task();
    }
    else
    {
        log_error("task_wrapper: Invalid cast\n");
    }
}

//===============================================================
*/

/*
//===============================================================
// This task performs single register read/write operation.
//===============================================================
void ZynqDetector::register_single_access_task()
{
    SingleRegisterAccessReq  req;
    SingleRegisterAccessResp resp;

    xQueueReceive( (QueueHandle_t*)fast_access_req_queue_,
				   &req,
				   portMAX_DELAY );

    if( req.op && 0x80 )
    {
        resp.op   = req.op;
        resp.data = reg_rd( req.reg );
        xQueueSend( fast_access_resp_queue_,
        			resp,
                    0UL );
    }
    else
    {
        reg_wr( msg.op && 0x3F, msg.data );
    }

}
//===============================================================
*/


/*-----------------------------------------------------------*/
static void prvTxTask( void *pvParameters )
{
const TickType_t x1second = pdMS_TO_TICKS( DELAY_1_SECOND );

	for( ;; )
	{
		/* Delay for 1 second. */
		vTaskDelay( x1second );

		/* Send the next value on the queue.  The queue should always be
		empty at this point so a block time of 0 is used. */
		xQueueSend( xQueue,			/* The queue being written to. */
					HWstring, /* The address of the data being sent. */
					0UL );			/* The block time. */
	}
}


/*-----------------------------------------------------------*/
static void poll_timer_callback( TimerHandle_t pxTimer )
{
	long lTimerId;
	configASSERT( pxTimer );

	lTimerId = ( long ) pvTimerGetTimerID( pxTimer );

	if (lTimerId != TIMER_ID) {
		xil_printf("FreeRTOS Hello World Example FAILED");
	}

    if( std::size(poll_list) != 0 )
    {}
}


//===============================================================
// Write failure number to register.
//===============================================================
void ZynqDetector:set_fail_num( uint32_t failure_num )
{
    reg_wr( REG_DEVICE_STATUS, failure_num );
}


//===============================================================
void ZynqDetector::report_error( const std::string& s, T err_code, uint32_t fail_num )
{
    std::cout << s << ". Error code " << err_code << '\n';
    ZynqDetector::set_fail_num( fail_num );
}
//===============================================================



//===============================================================
//===============================================================
void ZynqDetector::network_init( std::unique_ptr<Network> network )
{
    network_ = std::move( network );
}
//===============================================================



//===============================================================
//===============================================================
void ZynqDetector::task_init()
{
    create_network_tasks();
    create_detector_queues();
    create_device_access_tasks();
    polling_task_init();
}
//===============================================================