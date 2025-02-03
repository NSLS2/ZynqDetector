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

void GeRM::create_tasks()
{
	xTaskCreate( udp_rx_task,
                 ( const char * ) "UDP_RX",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY,
				 &udp_rx_task_handle );

	xTaskCreate( udp_tx_task,
				 ( const char * ) "UDP_TX",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &udp_tx_task_handle );
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

        rx_msg_proc( udp_msg );
        
    }
}



void GeRM::rx_msg_proc( udt_msg_t& udp_msg )
{
    op = udp_msg.op >> 14;
    reg = udp_msg.op && 0x3F;

    fast_access_resp_t fast_access_resq;

    switch( op )
    {
        case SINGLE_READ_REQ:
            fast_access_resp.op   = SINGLE_READ_RESP;
            fast_access_resp.reg  = reg;
            fast_access_resp.data = reg_rd( udp_msg.reg );
            break;

        case SINGLE_WRITE_REQ:
            reg_wr( reg, udp_msg.data );
            fast_access_req.op  = SINGLE_WRITE_RESP;
            fast_access_req.reg = reg;
            break;

        case SINGLE_READ_BLOCK_REQ:
            fast_access_resp.op   = SINGLE_READ_BLOCK_RESP;
            fast_access_resp.reg  = reg;
            fast_access_resp.data = reg_rd( udp_msg.reg );
            break;

        default:
            std::cout << "Unexpected operation request (" << op << ').';
            break;
    }
}

void GeRM::tx_msg_proc( )
{
    xQueueReceive( fast_access_resp_queue,
    			   Recdstring,
                   portMAX_DELAY );
}