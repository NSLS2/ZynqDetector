/**
 * @file Network.tpp
 * @brief Member function definitions of `Network`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

extern "C" {
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "lwip/init.h"
}

#include "ff.h"  // FatFS

//===========================================================================//


// To resolve the conflictions of read/write definition in socket.h and sstream
#ifdef read
#undef read
#endif

#ifdef write
#undef write
#endif

#include <stdexcept>
#include <sstream>
#include <string>

#include "Logger.hpp"
#include "task_wrap.hpp"

//===========================================================================//

/**
 * @brief Network constructor.
 * @param logger Reference to the logger.
 */
template < typename DerivedNetwork >
Network<DerivedNetwork>::Network( const Logger& logger )
	                            : logger_ ( logger )
{}

//===========================================================================//

/**
 * @brief Network initialization.
 */
template < typename DerivedNetwork >
void Network<DerivedNetwork>::network_init()
{
    int sock;

    ///< Initialize lwIP stack (TCP/IP thread + netif)
    lwip_init();
    //platform_init();


    ///< Add network interface to the netif_list, and set it as default.
    if (!xemac_add( &netif_,
                    NULL,
                    NULL,
                    NULL,
                    mac_addr_,
                    XPAR_XEMACPS_0_BASEADDR ))
    {
        xil_printf("Error adding network interface\r\n");
        return;
    }

    netif_set_default( &netif_ );

    ///< Specify that the network if is up.
    netif_set_up( &netif_ );

    read_network_config( "config" );


    if ( ( sock = socket( AF_INET, SOCK_DGRAM, 0 ) ) < 0 )
    {
        logger_.log_error( "Failed to create socket." );
        return;
    }

    memset( &local_addr_, 0, sizeof(struct sockaddr_in) );
    local_addr_.sin_family      = AF_INET;
    local_addr_.sin_port        = htons(UDP_PORT);
    local_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

    if ( bind( sock, (struct sockaddr *)&local_addr_, sizeof(local_addr_) != ERR_OK) )
    {
        logger_.log_error( "Error on bind" );
        close( sock );
        return;
    }

}

//===========================================================================//

/**
 * @brief Read network configuration parameters from file.
 * @details Read the following parameters:
 *             - IP address
 *             - Netmask
 *             - Gateway
 *             - MAC address
 * @param filename Name of the configuration file.
 */
//===============================================================
template < typename DerivedNetwork >
void Network<DerivedNetwork>::read_network_config( const std::string& filename )
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

    res = f_open(&file, filename.c_str(), FA_READ | FA_WRITE);
    if (res != FR_OK)
    {
        throw std::runtime_error( "Failed to open config file" );
    }


    //std::ifstream file( filename );
    //if ( !file.is_open() )
    //{
    //    throw std::runtime_error( "Failed to open config file" );
    //    std::cerr << "Error: could not open " << filename << "!\n";
    //    return;
    //}

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
        buff_index = 0;

        std::istringstream stream( buff );
        std::string key, value;

        stream >> key >> value;

        //if ( key == "ip-address" && !string_to_addr(value, ip_addr.begin() ) )
        //{
        //    throw NetException( "Invalid IP address format", value );
        //}
        //else if ( key == "netmask" && !string_to_addr( value, netmask.begin() ) )
        //{
        //    throw NetException( "Invalid netmask format", value );
        //}
        //else if ( key == "gateway" && !string_to_addr( value, gateway.begin() ) )
        //{
        //    throw NetException( "Invalid gateway format", value );
        //}
        //else if ( key == "dns" && !string_to_addr( value, dns.begin() ))
        //{
        //    throw NetException( "Invalid DNS format", value );
        //}
        //else if ( key == "mac-address" && !string_to_addr( value, mac_addr.begin() ) )
        //{
        //    throw NetException( "Invalid MAC address format", value );
        //}

        if ( key == "ip-address" )
        {
            if ( !inet_aton(value.c_str(), &netif_.ip_addr ) )
            {
                logger_.log_error( "Invalid IP address format", value );
                break;
            }
        }
        else
        {
            if ( key == "netmask" )
            {
                if ( !inet_aton( value.c_str(), &netif_.netmask ) )
                {
                    logger_.log_error( "Invalid netmask format", value );
                    break;
                }
            }
            else
            {
                if ( key == "gateway" )
                {
                    if ( !inet_aton( value.c_str(), &netif_.gw ) )
                    {
                        logger_.log_error( "Invalid gateway format", value );
                        break;
                    }
                }
                else
                {
                    if ( key == "mac-address" )
                    {
                        if ( !string_to_addr( value, mac_addr_ ) )
                        {
                            logger_.log_error( "Invalid MAC address format", value );
                            break;
                        }
                    }
                }
            }
        }

        memset( buff, 0, sizeof( buff ) );
    }

    f_close( &file );
    f_mount(NULL, "", 1);
}

//===========================================================================//

/**
 * @brief Convert a string to an IP/MAC address.
 * @param addr_str The string containing address.
 * @param addr The address from the string.
 * @return If the string contains a valid address.
 */
//===============================================================
//
//===============================================================
template < typename DerivedNetwork >
bool Network<DerivedNetwork>::string_to_addr(
    const std::string& addr_str,
    uint8_t* addr
)
{
    std::stringstream ss( addr_str );

    bool is_ip = ( addr_str.find( '.' ) != std::string::npos );
    auto separator = is_ip ? '.' : ':';
    auto num_separator = std::count( addr_str.begin(), addr_str.end(), separator );
    if ( ( is_ip && num_separator != 3 ) || ( !is_ip && num_separator != 5 ) )
    {
        logger_.log_error( "Wrong address string format", addr_str );
        return false;
    }

    std::string segment;

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

//===========================================================================//

/**
 * @brief Create network tasks (Rx and Tx).
 * @param
 */
template < typename DerivedNetwork >
void Network<DerivedNetwork>::create_network_tasks()
{
    rx_task_cfg_ = { .entry = [](void* ctx) { static_cast<Network<DerivedNetwork>*>(ctx)->udp_rx_task(); },
                     .context = this
                   };

    xTaskCreateStatic( task_wrapper
                     , "UDP Rx"
                     , RX_TASK_STACK_SIZE
                     , &rx_task_cfg_
                     , RX_TASK_PRIORITY
                     , rx_task_stack_
                     , &rx_task_tcb_
                     );

    tx_task_cfg_ = { .entry = [](void* ctx) { static_cast<Network<DerivedNetwork>*>(ctx)->udp_tx_task(); },
                     .context = this
                   };

    xTaskCreateStatic( task_wrapper
                     , "UDP Tx"
                     , TX_TASK_STACK_SIZE
                     , &tx_task_cfg_
                     , TX_TASK_PRIORITY
                     , tx_task_stack_
                     , &tx_task_tcb_
                     );
}

//===========================================================================//

/**
 * @brief UDP Rx task function.
 */
template < typename DerivedNetwork >
void Network<DerivedNetwork>::udp_rx_task()
{
    UdpRxMsg msg;
    int      msg_leng;
    
    struct sockaddr_in remote_addr;
    socklen_t remote_addr_leng = sizeof( remote_addr );

    uint32_t remote_ip_addr, remote_ip_addr_tmp;

    while(1)
    {
        msg_leng = recvfrom( udp_socket_
                           , &msg
                           , sizeof( msg )
                           , 0
                           , ( struct sockaddr* ) &remote_addr
                           , &remote_addr_leng );

        if ( (msg_leng <= 0) || (msg.id != UDP_REQ_MSG_ID) )
        {
            logger_.log_error( "Invalid UDP message received. Message ID = ", msg.id );
            continue;
        }

        remote_ip_addr_tmp = remote_addr.sin_addr.s_addr;
        if ( remote_ip_addr_tmp != remote_ip_addr )
        {
            ///< update server IP address
            remote_ip_addr = remote_ip_addr_tmp;
            remote_ip_addr_.store( remote_ip_addr_tmp, std::memory_order_relaxed );
        }
        
        rx_msg_proc( msg );
    }
}

//===========================================================================//

/**
 * @brief UDP Tx task function.
 */
template < typename DerivedNetwork >
void Network<DerivedNetwork>::udp_tx_task()
{
    struct sockaddr_in remote_addr;
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(UDP_PORT);

    uint16_t msg_leng;
    int      tx_length;
    UdpTxMsg msg;

    while(1)
    {
        msg_leng = static_cast<DerivedNetwork*>(this)->tx_msg_proc(msg);

        remote_addr.sin_addr.s_addr = remote_ip_addr_.load(std::memory_order_relaxed);

        tx_length = sendto( udp_socket_
                          , &msg
                          , msg_leng
                          , 0
                          , (struct sockaddr*)&remote_addr
                          , sizeof(remote_addr) );

        if (tx_length < 0)
        {
            logger_.log_error( "Failed to send UDP message\n" );
        }
    }
}

//===========================================================================//

/**
 * @brief UDP Tx message process.
 * @param msg UDP message to be sent.
 */
template< typename DerivedNetwork >
size_t Network<DerivedNetwork>::tx_msg_proc( UdpTxMsg& msg )
{
    return static_cast<DerivedNetwork*>(this)->tx_msg_proc_special( msg );
}

//===========================================================================//

/**
 * @brief UDP Rx message processing.
 * @param msg Received UDP message.
 */
template < typename DerivedNetwork >
void Network<DerivedNetwork>::rx_msg_proc( const UdpRxMsg& msg )
{
    static_cast<DerivedNetwork*>(this)->rx_msg_proc_special( msg );
}

//===========================================================================//
