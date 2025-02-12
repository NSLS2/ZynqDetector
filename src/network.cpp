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



//===============================================================
// Detector specific received UDP message processing.
//===============================================================
void DummyDetector::rx_msg_proc( udp_req_msg_t& msg )
{
    //==================================================
    // This function definition is for reference only.
    // A derived class must override.
    throw std::runtime_error( __PRETTY_FUNCTION__
        + " for reference only. Implement it for the derived class." );
    //==================================================
    
    switch( msg.op )
    {
        reg_access_req_t               reg_access_req;
        interface_single_access_req_t  interface_single_access_req;
        interface_multi_access_req_t   interface_multi_access_req;
        
        case READ_VERSION:
            reg_access_req.op = msg.op;
            reg_access_req.read = true;
            reg_access_req.reg_addr = VERSIONREG;

            xQueueSend( reg_access_req_queue,
         				reg_access_req,
                        0UL );
            break;

        case WRITE_AD5254_IOCTRL:
            //interface_single_access_req.op            = msg.op; // not needed for writing
            interface_single_access_req.write         = true;
            interface_single_access_req.ctrl_reg_addr = I2C0_CTRL_REG;
            interface_single_access_req.data_reg_addr = I2C0_DATA_REG;
            interface_single_access_req.data          = msg.data;

            xQueueSend( slow_access_req_queue,
        				slow_access_req,
                        0UL );
            break;

        case CONFIG_ASIC:
            interface_multi_access_req.op             = msg.op;
            interface_single_access_req.write         = true;
            interface_multi_access_req.ctrl_reg_addr  = SPI1_CTRL_REG;
            interface_single_access_req.data_reg_addr = I2C0_DATA_REG;
            interface_single_access_req.data          = msg.data;

            xQueueSend( bulk_access_req_queue,
        				bulk_access_req,
                        0UL );
            break;

        default:
            ;
        
    }
}
//===============================================================






//===============================================================
// Compose and send UDP message.
//===============================================================
void DummyDetector::tx_msg_proc( udp_tx_msg_t& msg )
{
    fast_access_resp_t fast_access_resp;
    i2c_access_resp_t  i2c0_access_resp;
    spi_access_resp_t  spi0_access_resp;

    udp_msg_t msg;
    msg.id = UDP_MSG_ID;
    uint32_t tx_leng;
    uint32_t msg_leng;
    
    active_resp_queue = xQueueSelectFromSet( resp_queue_set, portMAX_DELAY );

    if ( active_resp_queue == fast_access_resp_queue )
    {
        xQueueReceive( reg_access_resp_queue,
			           (void*)reg_access_resp,
                       portMAX_DELAY );
        msg.op = reg_access_resp.op;
        msg_leng = 4;
        memcpy( &msg.data, reg_access_resp.data, 4 );
    }
    else if ( active_resp_queue == interface_single_access_resp_queue )
    {
        xQueueReceive( interface_single_access_resp_queue,
                       (void*)interface_single_access_resp,
                       portMAX_DELAY );
        msg.op = interface_single_access_resp.op;
        msg.leng = 4;
        memcpy( &msg.data, interface_single_access_resp.data, 4 );
    }
    else if( active_resp_queue == interface_single_access_resp_queue )
    {
        xQueueReceive( interface_multi_access_resp_queue,
		               (void*)interface_multi_access_resp,
                       portMAX_DELAY );
        msg.op = interface_multi_access_resp.op;
        msg.leng = interface_multi_access_resp.leng;
        memcpy( &msg.data, interface_multi_access_resp.data, msg.leng );
    }
    else
    {
        // error
        std::err << "Invalid message queue." << '\n';
        continue;
    }
    
}
