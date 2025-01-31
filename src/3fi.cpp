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
            case SINGLE_READ_REQ:
                xQueueSend( fast_access_req_queue,
            				fast_access_req,
                            0UL );
                break;

            case SINGLE_WRITE_REQ:
                xQueueSend( slow_access_req_queue,
            				slow_access_req,
                            0UL );
                break;

            case SINGLE_READ_BLOCK_REQ:
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

        if ( active_resp_queue == fast_access_resp_queue )
        {

        }
        else
        {
            if ( active_resp_queue == slow_access_resp_queue )
            {

            }
            else // active_resp_queue == bulk_access_resp_queue
            {
                
            }
        }
    }

}

void GeRM::rx_msg_proc( udt_msg_t& udp_msg )
{
    op = udp_msg.op >> 14;
    reg = udp_msg.op && 0x3F;

    switch( op )
    {
        case SINGLE_READ_REQ:
            fast_access_req.op   = SINGLE_READ_RESP;
            fast_access_req.reg  = reg;
            fast_access_req.data = reg_rd( udp_msg.reg );
            
            break;
        case SINGLE_WRITE_REQ:
            reg_wr( reg, udp_msg.data );
            break;
        case SINGLE_READ_BLOCK_REQ:
            fast_access_resp.op   = SINGLE_READ_BLOCK_RESP;
            fast_access_resp.reg  = reg;
            fast_access_resp.data = reg_rd( udp_msg.reg );
            break;
        case READ_REG:
            break;
    }
}