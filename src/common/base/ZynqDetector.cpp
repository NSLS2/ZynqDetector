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
    //std::cout << __PRETTY_FUNCTION__
    //    << " for reference only. Implement it for the specific detector.\n" );
    log_error("For reference only. Implement it for the specific detector.\n" );
    exit();
}


//===============================================================
// Network initialization.
//===============================================================
ZynqDetector::network_init()
{
    read_network_config( "config" );

    struct freertos_sockaddr sock_addr;
    sock_addr.sin_port = FreeRTOS_htons( 25913 );
    sock_addr.sin_addr = FreeRTOS_inet_addr( ip_addr_ );

    // Initialize the FreeRTOS+TCP stack
    FreeRTOS_IPInit( ip_addr_, netmask_, gateway_, dns_, mac_address_ );

    // Create a UDP socket
    udp_socket_ = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP );

    if ( socket < 0 )
    {
        throw std::runtime_error( "Failed to create socket!" );
        //std::cerr << "Failed to create socket (error )" << socket << '\n';
    }

    // Bind the socket to the UDP port
    if ( FreeRTOS_bind( socket, &sock_addr, sizeof( sock_addr ) ) < 0 )
    {
        throw std::runtime_error( "Failed to bind the socket to the port!" );
        //std::cerr << "Failed to bind the socket to the port (error )" << socket << '\n';
    }
}
//==============================================================

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
// UDP receive task.
//===============================================================
void ZynqDetector::udp_rx_task( void *pvParameters )
{
    UDPRxMsg msg;
    uint32_t msg_leng;
    
    struct freertos_sockaddr src_sock_addr;
    socklen_t src_addr_leng = sizeof( src_sock_addr );

    uint32_t remote_ip_addr, remote_ip_addr_tmp;

    //=============================
    // Initialize network
    //=============================
    xUDPSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);

    while(1)
    {
        uint16_t msg_leng = FreeRTOS_recvfrom( udp_socket_,
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
            memcpy( svr_ip_addr_, &remote_ip_addr, 4 );
        }
        
        rx_msg_proc( msg );
    }

}
//===============================================================



//===============================================================
// UDP Rx message processing.
//===============================================================
void ZynqDetector::rx_msg_proc( std::any& msg )
{
    int instr = msg.op && 0x8F;
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
//===============================================================
