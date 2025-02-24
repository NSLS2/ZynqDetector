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
ZynqDetector::ZynqDetector( uint32_t base_addr )
    : base_addr_( base_addr )
{}


//===============================================================
//  Single register access.
//  Assembles fast access request and sends it to the queue.
//===============================================================
void single_reg_acc_req_proc( udp_rx_msg_t& msg )
{
    fast_access_req_t req;
    req.op   = msg->op;
    req.data = msg->data;
    xQueueSend( fast_access_req_queue,
            	req,
                0UL );
}
//===============================================================


/*
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
        xQueueReceive( 	static_cast<(QueueHandle_t*>(pvParameters),
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
*/

//===============================================================
// This task performs single register read/write operation.
//===============================================================
void ZynqDetector::single_register_access_task()
{
    SingleRegisterAccessReq  req;
    SingleRegisterAccessResp resp;

    xQueueReceive( (QueueHandle_t*)fast_access_req_queue,				/* The queue being read. */
				   &req,	/* Data is read into this address. */
				   portMAX_DELAY );	/* Wait without a timeout for data. */

    if( msg.op && 0x80 )
    {
        resp.op   = msg.op;
        resp.data = reg_rd( msg.reg );
        xQueueSend( fast_access_resp_queue,
        			resp,
                    0UL );
    }
    else
    {
        reg_wr( msg.op && 0x3F, msg.data );
    }

}


//===============================================================
// This task performs single register read/write operation.
//===============================================================
void ZynqDetector::interface_single_access_task( void *pvParameters )
{
    interface_single_access_req_t  req;
    interface_single_access_resp_t resp;

    auto param = static_cast<interface_single_access_task_param*>(pvParameters);

    while(1)
    {
        xQueueReceive( 	static_cast<QueueHandle_t*>(param).req_queue,
						&req,
						portMAX_DELAY );
        
        if ( req.read )
        {
            interface_read( req.device_addr, req.data );
            resp.op = req.op;

            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            resp.data = reg.rd( req.addr );

            xQueueSend( static_cast<QueueHandle_t*>(param).resp_queue,
                        static_cast<const void*>(resp),
                        portMAX_DELAY );
        }
    }
}
//===============================================================

//===============================================================
// This task processes requests that need to continuously 
// access a device multiple times, e.g., to configure a device
// through an I2C bus while the configuration data consists of
// multiple dwords.
//===============================================================
void ZynqDetector::interface_multi_access_task( void *pvParameters )
{
    interface_multi_access_req_t  req;
    interface_multi_access_resp_t resp;

    while(1)
    {
        xQueueReceive( 	static_cast<QueueHandle_t*>(param).req_queue,
						&req,
                        portMAX_DELAY );

        // compose the instruction queue
        
        for ( int i=0; !instr_queue.empty(); ++i )
        {
            auto instr = instr_queue.front();
            instr_queue.pop();
            if ( instr.read )
            {
                read( instr.device_addr, instr.reg_addr );
            else
            {
                write( instr.device_addr, instr.reg_addr, instr.data );
            }

            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if ( instr.read )
            {
                resp.data[i] = reg.rd( req.addr );
            }
        }

        if( req.read )
        {
            resp.op = req.op;
            
            xQueueSend( static_cast<QueueHandle_t*>(param).resp_queue,
                        static_cast<const void*>(resp),
                        portMAX_DELAY );
        }
    }
}
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
void ZynqDetector::queue_init()
{
    //==================================================
    // This function definition is for reference only.
    // A derived class must override.
    throw std::runtime_error( __PRETTY_FUNCTION__
        + " for reference only. Implement it for the derived class." );
    //==================================================

    // Create queues        
	reg_access_req_queue               = xQueueCreate( 100, sizeof( reg_access_req_t ) );
	interface_single_access_req_queue  = xQueueCreate( 100, sizeof( interface_single_access_req_t ) );
    interface_multi_access_req_queue   = xQueueCreate( 4,   sizeof( interface_multi_access_req_t ) );
	reg_access_resp_queue              = xQueueCreate( 100, sizeof( reg_access_resp_t ) );
	interface_single_access_resp_queue = xQueueCreate( 100, sizeof( interface_single_access_resp_t ) );
    interface_multi_access_resp_queue  = xQueueCreate( 4,   sizeof( interface_multi_access_resp_t ) );

    // Create queue sets
    resp_queue_set = xQueueCreateSet( 
        REG_ACCESS_RESP_QUEUE_SIZE +
        INTERFACE_SINGLE_ACCESS_RESP_QUEUE_SIZE +
        INTERFACE_MULTI_ACCESS_RESP_QUEUE_SIZE );

    xQueueAddToSet( reg_access_resp_queue, resp_queue_set );
    xQueueAddToSet( interface_single_access_resp_queue, resp_queue_set );
    xQueueAddToSet( interface_multi_access_resp_queue, resp_queue_set );
}


//===============================================================
//===============================================================
void ZynqDetector:task_init()
{
    //==================================================
    // This function definition is for reference only.
    // A derived class must override.
    throw std::runtime_error( __PRETTY_FUNCTION__
        + " for reference only. Implement it for the derived class." );
    //==================================================

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

	xTaskCreate( reg_access_task,
				 ( const char * ) "REG_ACCESS",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &reg_access_task_handle );

    xTaskCreate( interface_signle_access_task,
                 ( const char * ) "SLOW_ACCESS",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 &interface_single_access_task_handle );

	xTaskCreate( interface_multi_access_task,
				 ( const char * ) "SLOW_ACCESS",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &interface_multi_access_task_handle );
}
//===============================================================



//===============================================================
// UDP receive task.
//===============================================================
void ZynqDetector::udp_rx_task( void *pvParameters )
{
    udp_req_msg_t msg;
    uint32_t msg_leng;
    
    struct freertos_sockaddr src_sock_addr;
    socklen_t src_addr_leng = sizeof(src_sock_addr);

    uint32_t remote_ip_addr, remote_ip_addr_tmp;

    //=============================
    // Initialize network
    //=============================
    xUDPSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);

    while(1)
    {
        uint16_t msg_leng = FreeRTOS_recvfrom( xSocket,
                                               &msg,
                                               sizeof( msg ),
                                               0,
                                               ( struct freertos_sockaddr * ) &src_sock_addr,
                                               &src_addr_leng );

        if ( (msg_leng <= 0) || (msg.id != UDP_MSG_ID) )
        {
            // error report
            continue;
        }

        remote_ip_addr_tmp = FreeRTOS_ntohl(src_addr.sin_addr);
        if ( remote_ip_addr_tmp != remote_ip_addr )
        {
            // update server IP address
            remote_ip_addr = remote_ip_addr_tmp;
            
        }

        
        rx_msg_proc( msg );
    }

}
//===============================================================



//===============================================================
// UDP transmit task.
//===============================================================
void ZynqDetector::udp_tx_task( void *pvParameters )
{
    //==================================================
    // This function definition is for reference only.
    // A derived class must override.
    throw std::runtime_error( __PRETTY_FUNCTION__
        + " for reference only. Implement it for the derived class." );
    //==================================================
    
    struct freertos_sockaddr dest_sock_addr;
    socklen_t dest_sock_addr_leng = sizeof(xDestination);
    int32_t bytesSent;

    dest_sock_addr.sin_addr = FreeRTOS_inet_addr_quick( svr_ip_addr[3],
                                                        svr_ip_addr[2],
                                                        svr_ip_addr[1],
                                                        svr_ip_addr[0] );
    dest_sock_addr.sin_port = FreeRTOS_htons(UDP_PORT);

    udp_tx_msg_t msg;
    
    while(1)
    {
        tx_msg_proc( msg );
        tx_leng = FreeRTOS_sendto( udp_socket,
                                   msg,
                                   msg.leng + 4, // ID (2) + OP (2) + data
                                   0,
                                   &dest_sock_addr,
                                   dest_sock_addr_leng );
        if ( tx_leng <= 0 )
        {
            std::err << "Failed to send UDP message.\n";
        }
    }
}
//===============================================================



//===============================================================
// Convert a string to an IP/MAC address.
//===============================================================
bool ZynqDetector::string_to_addr( const std::string& addr_str, uint8_t* addr )
{
    std::stringstream ss( addr_str );

    bool is_ip = ( addr_str.find( '.' ) != std::string::npos );
    auto separator = is_ip ? '.' : ':';
    auto num_separator = std::count( addr_str.begin(), addr_str.end(), separator );
    if ( ( is_ip && num_separator != 3 ) || ( !is_ip && num_separator != 5 ) )
    {
        std::cerr << "Wrong address string format" << addr_str << '\n';
        return false;
    }

    std::string segment;

    int i = 0;

    while ( std::getline ( ss, segment, separator ) )
    {
        int byte = 0;
        if ( is_ip )
        {
            if ( std::stringstream( segment ) >> byte )
            {
                if (byte < 0 || byte > 255) return false;  // Invalid byte value
            }
            else
            {
                return false;  // Invalid segment
            }
        }
        else
        {
            try
            {
                byte = std::stoi( segment, nullptr, 16 );
                if (byte < 0 || byte > 255) return false;  
            }
            catch ( const std::invalid_argument& )
            {
                return false;  // Invalid hex segment
            }            
        }
        *(addr++) = static_cast<uint8_t>( byte );
    }

    return true;
}

//===============================================================
// Read network parameters from file:
// - IP address
// - Netmask
// - Gateway
// - DNS server
// - MAC address
//===============================================================

void ZynqDetector::read_network_config( const std::string& filename )
{
    FATFS fs;    // File system object
    FRESULT res; // Result code
    FIL file;
    UINT br;

    char buff[50];
    int buff_index = 0;
    
    res = f_mount(&fs, "", 1); // Mount the default drive
    if (res != FR_OK)
    {
        throw std::runtime_error( "Failed to mount SD card" );
    }

    res = f_open(&file, "filename.txt", FA_READ | FA_WRITE);
    if (res != FR_OK)
    {
        throw std::runtime_error( "Failed to open config file" );
    }


    std::ifstream file( filename );
    if ( !file.is_open() )
    {
        throw std::runtime_error( "Failed to open config file" );
        std::cerr << "Error: could not open " << filename << "!\n";
        return;
    }

    std::string line;

    //while( std::getline(file, line) )
    while (f_read(&file, &buff[buff_index], 1, &br) == FR_OK && br > 0)
    {
        if ( buff[buff_index] != '\n' && buff[buff_index] != '\r')
        {
            ++buff_index;
            continue;
        }
        
        buff[buff_index] = '\0'; // Null-terminate the line

        std::istringstream stream( buff );
        std::string key, value;

        stream >> key >> value;
        if ( key == "ip-address" && !string_to_addr(value, ip_addr.begin() ) )
        {
            throw NetException( "Invalid IP address format", value );
        }
        else if ( key == "netmask" && !string_to_addr( value, netmask.begin() ) )
        {
            throw NetException( "Invalid netmask format", value );
        }
        else if ( key == "gateway" && !string_to_addr( value, gateway.begin() ) )
        {
            throw NetException( "Invalid gateway format", value );
        }
        else if ( key == "dns" && !string_to_addr( value, dns.begin() ))
        {
            throw NetException( "Invalid DNS format", value );
        }
        else if ( key == "mac-address" && !string_to_addr( value, mac_addr.begin() ) )
        {
            throw NetException( "Invalid MAC address format", value );
        }

        memset( buff, sizeof( buff ), 0 );
        buff_index = 0;
    }

    f_close( &file );
    f_mount(NULL, "", 1);
}
//===============================================================


//===============================================================
// Network initialization.
//===============================================================
ZynqDetector::network_init()
{
    read_network_config( "config" );

    struct freertos_sockaddr sock_addr;
    sock_addr.sin_port = FreeRTOS_htons( 25913 );
    sock_addr.sin_addr = FreeRTOS_inet_addr( ip_addr );

    // Initialize the FreeRTOS+TCP stack
    FreeRTOS_IPInit( ip_address, netmask, gateway, dns, mac_address );

    // Create a UDP socket
    int32_t socket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP );

    if ( socket < 0 )
    {
        throw std::runtime_error( "Failed to create socket!" );
        //std::cerr << "Failed to create socket (error )" << socket << '\n';
    }

    // Bind the socket to the UDP port
    if ( FreeRTOS_bind( socket, &sock_addr, sizeof(sock_addr)) < 0 )
    {
        throw std::runtime_error( "Failed to bind the socket to the port!" );
        //std::cerr << "Failed to bind the socket to the port (error )" << socket << '\n';
    }
}
//==============================================================


void ZynqDetector::rx_msg_proc(int instr, std::any& msg) {
    auto it = instr_map_.find(instr);
    if (it != instr_map_.end())
    {
        it->second(msg);  // Call the corresponding function
    }
    else
    {
        std::cout << "Unknown instruction: " << instr << '\n';
    }
}