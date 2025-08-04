#pragma once

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

//#include "lwipopts.h"
//#include "lwip/ip_addr.h"
//#include "lwip/err.h"
//#include "lwip/udp.h"
//#include "lwip/sys.h"
//#include "lwip/init.h"

#include "netif/xadapter.h"
#include "errno.h"
#include "xparameters.h"
}

//#include "ZynqDetector.hpp"
#include "Logger.hpp"

template < typename DerivedNetwork >
class Network
{

public:

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
    void network_init();
    void create_network_tasks();

private:
    const Logger& logger_;

protected:

    struct netif netif_;
    int    sock_;
    struct sockaddr_in local_addr_;

    //uint8_t ip_addr_[4];
    //uint8_t netmask_[4];
    //uint8_t gateway_[4];
    //uint8_t dns_[4];
    uint8_t mac_addr_[6];

//    socket_t xUDPSocket;

    alignas(64) std::atomic<uint32_t> remote_ip_addr_;

    int32_t udp_socket_;

    using MessageHandler = std::function<void(std::any&)>;
    std::map<int, MessageHandler> rx_msg_map_;

    void msg_map_init();

    void read_network_config( const std::string& filename );
    static void tcpip_init_done( void *arg );
    bool string_to_addr( const std::string& addr_str, uint8_t* addr );

    TaskHandle_t udp_rx_task_handle_;
    TaskHandle_t udp_tx_task_handle_;
    
    //void create_network_tasks();
    void udp_rx_task();
    void udp_tx_task();

    void rx_msg_proc( UdpRxMsg& msg );
    //virtual void tx_msg_proc() = 0;
    

};
 
 #include "Network.tpp"
