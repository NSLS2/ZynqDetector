// C++ includes
#include <iterator>
#include <portmacro.h>
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
#include "zynq_detector.hpp"
#include "pynq_ssd_msg.hpp"

#define TIMER_ID	1
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define TIMER_CHECK_THRESHOLD	9
/*-----------------------------------------------------------*/

/* The Tx and Rx tasks as described at the top of this file. */
/*-----------------------------------------------------------*/

/* The queue used by the Tx and Rx tasks, as described at the top of this
file. */
static TaskHandle_t xTxTask;
static TaskHandle_t xRxTask;
static QueueHandle_t xQueue = NULL;
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


void ZynqDetector::fast_access_task( void *pvParameters )
{
    fast_access_req_t fast_access_req;

    while(1)
    {
        xQueueReceive( 	(QueueHandle_t*)fast_access_req_queue,				/* The queue being read. */
						&fast_access_req,	/* Data is read into this address. */
						portMAX_DELAY );	/* Wait without a timeout for data. */
    }
}

void ZynqDetector::slow_access_task( void *pvParameters )
{
    active_slow_req_queue  = ( slow_req_queue_set, portMAX_DELAY );

}

void ZynqDetector::bulk_access_task( void *pvParameters )
{

}

ZynqDetector::ZynqDetector( void )
{
	const TickType_t x1second = pdMS_TO_TICKS( DELAY_1_SECOND );

    // Network
    network_init();


    //=============================
    // Queues
	//=============================
    // Create queues
	fast_access_req_queue  = xQueueCreate( 100, sizeof( fast_access_req_t ) );
	slow_access_req_queue  = xQueueCreate( 100, sizeof( slow_access_req_t ) );
    bulk_access_req_queue  = xQueueCreate( 4,   sizeof( bulk_access_req_t ) );
	fast_access_resp_queue = xQueueCreate( 100, sizeof( fast_access_resp_t ) );
	slow_access_resp_queue = xQueueCreate( 100, sizeof( slow_access_resp_t ) );
    bulk_access_resp_queue = xQueueCreate( 4,   sizeof( bulk_access_resp_t ) );

    // Create queue sets
    slow_req_queue_set  = xQueueCreateSet(
        SLOW_ACCESS_REQ_QUEUE_SIZE +
        BULK_ACCESS_REQ_QUEUE_SIZE );

    xQueueAddToSet( slow_access_req_queue, slow_req_queue_set );
    xQueueAddToSet( bulk_access_req_queue, slow_req_queue_set );

    resp_queue_set = xQueueCreateSet( 
        FAST_ACCESS_RESP_QUEUE_SIZE +
        SLOW_ACCESS_RESP_QUEUE_SIZE +
        BULK_ACCESS_RESP_QUEUE_SIZE );

    xQueueAddToSet( fast_access_resp_queue, resp_queue_set );
    xQueueAddToSet( slow_access_resp_queue, resp_queue_set );
    xQueueAddToSet( bulk_access_resp_queue, resp_queue_set );

    //=============================
    // Create tasks
	xTaskCreate( udp_rx_task, 				 // The function that implements the task.
                 ( const char * ) "UDP_RX",  // Text name for the task, provided to assist debugging only.
				 configMINIMAL_STACK_SIZE,   // The stack allocated to the task.
				 NULL, 					     // The task parameter is not used, so set to NULL.
				 tskIDLE_PRIORITY,			 // The task runs at the idle priority.
				 &udp_rx_task_handle );

	xTaskCreate( udp_tx_task,
				 ( const char * ) "UDP_TX",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &udp_tx_task_handle );

	xTaskCreate( fast_access_task,
				 ( const char * ) "FAST_ACCESS",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &fast_access_task_handle );

	xTaskCreate( slow_access_task,
				 ( const char * ) "SLOW_ACCESS",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &slow_access_task_handle );

	// Create a timer
	xPollTimer = xTimerCreate( (const char *) "Timer",
							   x1second,
							   pdTRUE,
							   (void *) TIMER_ID,
							   vTimerCallback);
	// Check timer creation
	configASSERT( xPollTimer );

	// start the timer
	xTimerStart( xPollTimer, 0 );

	// Start the tasks
	vTaskStartScheduler();


	for( ;; );
}


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
static void prvRxTask( void *pvParameters )
{
char Recdstring[15] = "";

	for( ;; )
	{
		/* Block to wait for data arriving on the queue. */
		xQueueReceive( 	xQueue,				/* The queue being read. */
						Recdstring,	/* Data is read into this address. */
						portMAX_DELAY );	/* Wait without a timeout for data. */

		/* Print the received data. */
		xil_printf( "Rx task received string from Tx task: %s\r\n", Recdstring );
		RxtaskCntr++;
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

void ZynqDetector::network_init()
{
    struct freertos_sockaddr sock_addr;
    sock_addr.sin_port = FreeRTOS_htons( 1234 );
    sock_addr.sin_addr = FreeRTOS_inet_addr( ip_addr );

    // Initialize the FreeRTOS+TCP stack
    FreeRTOS_IPInit( ip_address, netmask, gateway, dns, MACAddress);

    // Create a UDP socket
    int32_t socket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP );

    if (socket < 0) {
        // Handle error
    }

    // Bind the socket to the UDP port
    if (FreeRTOS_bind( socket, &sock_addr, sizeof(sock_addr)) < 0) {
        // Handle error
    }

}


void ZynqDetector::udp_rx_task( void *pvParameters )
{
    uint16_t op;
    uint16_t obj;
    udp_msg_t udp_msg;

    uint8_t obj_type = 0;

    fast_access_req_t fast_access_req;
    slow_access_req_t slow_access_req;
    bulk_access_req_t bulk_access_req;

    Socket_t xUDPSocket;
    struct freertos_sockaddr xSourceAddress;
    socklen_t xSourceAddressLength = sizeof(xSourceAddress);
    int32_t lBytesReceived;
    uint8_t ucBuffer[MAX_UDP_MSG_LENG];

    //=============================
    // Initialize network
    //=============================
    xUDPSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);

    while(1)
    {
        lReceivedBytes = FreeRTOS_recvfrom( xSocket, ucReceiveBuffer, sizeof( ucReceiveBuffer ), 0, ( struct freertos_sockaddr * ) &xClientAddress, &xClientAddressLength );

        // Read UDP packet
        op = udp_msg.op >> 14;
        uint16_t obj = udp_msg.msg_id & 0x3F;
        switch( obj )
        {
            case MSG_VER:
                // send a fast_req to fast_access_task
                obj_type = FAST_ACCESS_REQ;
                break;
            default:
                ;
        }

        switch( obj_type )
        {
            case FAST_ACCESS_REQ:
                xQueueSend( fast_access_req_queue,
            				fast_access_req,
                            0UL );
                break;

            case SLOW_ACCESS_REQ:
                xQueueSend( slow_access_req_queue,
            				slow_access_req,
                            0UL );
                break;

            case BULK_ACCESS_REQ:
                xQueueSend( bulk_access_req_queue,
            				bulk_access_req,
                            0UL );
                break;

            default:
                ;
                
        }
        
    }
}

void ZynqDetector::udp_tx_task( void *pvParameters )
{
    while(1)
    {
        active_resp_queue = ( resp_queue_set, portMAX_DELAY );

        if ( active_resp_queue == reg_access_resp_queue )
        {

        }
        else
        {
            if ( active_resp_queue == interface_single_access_resp_queue )
            {

            }
            else // active_resp_queue == interface_multi_access_resp_queue
            {
                
            }
        }
    }

}


// Write failure number to register.
void ZynqDetector:set_fail_num( uint32_t failure_num )
{
    reg_wr( REG_DEVICE_STATUS, failure_num );
}


void ZynqDetector::report_err( const std::string& s, T err_code, uint32_t fail_num )
{
    std::cout << s << ". Error code " << err_code << '\n';
    ZynqDetector::set_fail_num( fail_num );
}