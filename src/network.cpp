#include <iostream>
#include <sstream>
#include <stdexcept>
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
#include "zynqDetector.hpp"

//==============================================================

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

//==============================================================

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

//==============================================================

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
/*
Network::Network( ZynqDetector& detector )
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
        std::cerr << "Failed to create socket (error )" << socket << '\n';
    }

    // Bind the socket to the UDP port
    if ( FreeRTOS_bind( socket, &sock_addr, sizeof(sock_addr)) < 0 )
    {
        throw std::runtime_error( "Failed to bind the socket to the port!" );
        std::cerr << "Failed to bind the socket to the port (error )" << socket << '\n';
    }

}
*/

void ZynqDetector::udp_rx_task( void *pvParameters )
{
    udp_msg_t udp_msg;
    uint32_t udp_msg_leng;
    
    struct freertos_sockaddr src_sock_addr;
    socklen_t src_addr_leng = sizeof(src_sock_addr);

    uint32_t remote_ip_addr, remote_ip_addr_tmp;

    //=============================
    // Initialize network
    //=============================
    xUDPSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);

    while(1)
    {
        uint16_t udp_msg_leng = FreeRTOS_recvfrom( xSocket,
                                                   &udp_msg,
                                                   sizeof( udp_msg ),
                                                   0,
                                                   ( struct freertos_sockaddr * ) &src_sock_addr,
                                                   &src_addr_leng );

        if ( (udp_msg_leng <= 0) || (udp_msg.id != UDP_MSG_ID) )
        {
            // error report
            continue;
        }

        remote_ip_addr_tmp != FreeRTOS_ntohl(src_addr.sin_addr);
        if ( remote_ip_addr_tmp != remote_ip_addr )
        {
            // update server IP address
            remote_ip_addr = remote_ip_addr_tmp;
            
        }

        rx_msg_proc( udp_msg );
    }

}

void ZynqDetector::rx_msg_proc( udp_msg_t& msg )
{
    //==================================================
    // This function definition is for reference only.
    // A derived class must override.
    throw std::runtime_error( __PRETTY_FUNCTION__
        + " for reference only. Implement it for the derived class." );
    //==================================================
    
    switch( msg.op )
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

    tx_msg_proc( dest_sock_addr, dest_sock_addr_leng );
}

void ZynqDetector::tx_msg_proc( const struct freertos_sockaddr dest_sock_addr&,
                                const socklen_t dest_sock_addr_leng )
{
    //==================================================
    // This function definition is for reference only.
    // A derived class MUST override.
    throw std::runtime_error( __PRETTY_FUNCTION__
        + " for reference only. Override it in the derived class." );
    //==================================================

    fast_access_resp_t fast_access_resp;
    i2c_access_resp_t  i2c0_access_resp;
    spi_access_resp_t  spi0_access_resp;

    udp_msg_t msg;
    msg.id = UDP_MSG_ID;
    uint32_t tx_leng;
    uint32_t msg_leng;

    while(1)
    {
        active_resp_queue = xQueueSelectFromSet( resp_queue_set, portMAX_DELAY );

        if ( active_resp_queue == fast_access_resp_queue )
        {
            xQueueReceive( fast_access_resp_queue,
			               (void*)fast_access_resp,
                           portMAX_DELAY );
            msg.op = fast_access_resp.op;
            msg_leng = fast_access_resp.leng;
            memcpy( &msg.data, fast_access_resp.data, msg_leng );
        }
        else if ( active_resp_queue == slow_access_resp_queue )
        {
            xQueueReceive( slow_access_resp_queue,
                           (void*)slow_access_resp,
                           portMAX_DELAY );
            msg.op = slow_access_resp.op;
            msg_leng = fast_access_resp.leng;
            memcpy( &msg.data, slow_access_resp.data, msg_leng );
        }
        else if( active_resp_queue == bulk_access_resp_queue )
        {
            xQueueReceive( bulk_access_resp_queue,
		                   (void*)bulk_access_resp,
                           bulk_access_resp );
            msg.op = bulk_access_resp.op;
            msg_leng = fast_access_resp.leng;
            memcpy( &msg.data, bulk_access_resp.data, msg_leng );
        }
        else
        {
            // error
            std::err << "Invalid message queue." << '\n';
            continue;
        }

        tx_leng = FreeRTOS_sendto( udp_socket,
                                   msg,
                                   msg_leng + 4, // ID (2) + OP (2) + data
                                   0,
                                   &dest_sock_addr,
                                   dest_sock_addr_leng );
        if ( tx_leng <= 0 )
        {
            std::err << "Failed to send UDP message.\n";
        }
    }
}
