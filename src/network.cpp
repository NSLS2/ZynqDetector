#include <iostream>
#include <sstream>
#include <string>
#include <array>
#include <vector>
#include <cctype>
#include <cstdint>
#include <algorithm>

#include "ff.h"       // FatFs header
#include "FreeRTOS.h" // FreeRTOS header
#include "task.h"

#include "network.hpp"

//==============================================================

bool string_to_addr( const std::string& addr_str, uint8_t* addr )
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

//==============================================================




void network::read_network_config( const std::string& filename )
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
        // Handle error

    }

    res = f_open(&file, "filename.txt", FA_READ | FA_WRITE);
    if (res != FR_OK)
    {
        // Handle error
    }


    std::ifstream file( filename );
    if ( !file.is_open() )
    {
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
            std::cerr << "Invalid IP address format: " << value << '\n';
        }
        else if ( key == "netmask" && !string_to_addr( value, netmask.begin() ) )
        {
            std::cerr << "Invalid netmask format: " << value << '\n';
        }
        else if ( key == "gateway" && !string_to_addr( value, gateway.begin() ) ) {
            std::cerr << "Invalid gateway format: " << value << '\n';
        }
        else if ( key == "dns" && !string_to_addr( value, dns.begin() )) {
            std::cerr << "Invalid DNS format: " << value << '\n';
        }
        else if ( key == "mac-address" && !string_to_addr( value, mac_addr.begin() ) ) {
            std::cerr << "Invalid MAC address format: " << value << '\n';
        }

        memset( buff, sizeof( buff ), 0 );
        buff_index = 0;
    }

    f_close( &file );
    f_mount(NULL, "", 1);

}


network::network()
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