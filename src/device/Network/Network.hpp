/**
 * @file Network.hpp
 * @brief Class template definition of `Network`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <cstdint>
#include <memory>
#include <atomic>
#include <functional>
#include <any>
#include <map>

extern "C" {
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/inet.h"

#include "netif/xadapter.h"
#include "errno.h"
#include "xparameters.h"
}

#include "Logger.hpp"

//===========================================================================//

template < typename DerivedNetwork >
class Network
{

public:

    ///< UDP and message
    static constexpr uint32_t UDP_PORT        = 0x7000;
    static constexpr uint32_t UDP_REQ_MSG_ID  = 0xCAFE;
    static constexpr uint32_t UDP_RESP_MSG_ID = 0xBEEF;
    //------------------------------
    // UDP message
    //------------------------------
    static constexpr uint16_t MAX_UDP_MSG_LENG = 4096;
    static constexpr uint16_t MAX_UDP_MSG_DATA_LENG = MAX_UDP_MSG_LENG - 4; // length of message data in bytes

    struct UdpReqMsg
    {   
        uint16_t                           id; 
        uint16_t                           op;
        DerivedNetwork::UdpReqMsgPayload   payload;
    };  
    using UdpRxMsg = UdpReqMsg;

    struct UdpRespMsg
    {
        uint16_t                           id;
        uint16_t                           op;
        DerivedNetwork::UdpRespMsgPayload  payload;
    };
    using UdpTxMsg = UdpRespMsg;


    explicit Network( const Logger& logger  );

    /**
     * @brief Network initialization
     */
    void network_init();

    /**
     * @brief Create network tasks (Rx/Tx)
     */
    void create_network_tasks();

private:
    const Logger& logger_;

    struct netif netif_;
    struct sockaddr_in local_addr_;
    int32_t udp_socket_;
    uint8_t mac_addr_[6];
    alignas(64) std::atomic<uint32_t> remote_ip_addr_;

    static constexpr UBaseType_t  RX_TASK_PRIORITY   = 10;
    static constexpr uint32_t     RX_TASK_STACK_SIZE = 1000;
    StaticTask_t                  rx_task_tcb_;
    StackType_t                   rx_task_stack_[RX_TASK_STACK_SIZE];
    TaskConfig                    rx_task_cfg_;

    static constexpr uint32_t     TX_TASK_PRIORITY   = 9;
    static constexpr uint32_t     TX_TASK_STACK_SIZE = 1000;
    StaticTask_t                  tx_task_tcb_;
    StackType_t                   tx_task_stack_[RX_TASK_STACK_SIZE];
    TaskConfig                    tx_task_cfg_;

    TaskHandle_t udp_rx_task_handle_;
    TaskHandle_t udp_tx_task_handle_;
    
    /**
     * @brief Read network configuration parameters from file.
     */
    void read_network_config( const std::string& filename );

    /**
     * @brief Check if TCP/IP initialization is done.
     */
    static void tcpip_init_done( void *arg );

    /**
     * @brief Convert a string to an IP/MAC address.
     */
    bool string_to_addr( const std::string& addr_str, uint8_t* addr );

    /**
     * @brief UDP Rx task function.
     */
    void udp_rx_task();
    
    /**
     * @brief UDP Tx task function.
     */
    void udp_tx_task();

    /**
     * @brief UDP Rx message process.
     */
    void rx_msg_proc( const UdpRxMsg& msg );

    /**
     * @brief UDP Tx message process.
     */
    size_t tx_msg_proc( UdpTxMsg& msg );

};
 
 #include "Network.tpp"
