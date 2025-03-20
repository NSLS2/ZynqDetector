#pragma once

#include <cstdint>
#include <memory>

class Network
{
private:
    ZynqDetector* owner_;

protected:
    uint32_t udp_port_;
    const uint16_t MAX_UDP_MSG_LENG = 4096;
    const uint16_t MAX_UDP_MSG_DATA_LENG = MAX_UDP_MSG_LENG - 4; // length of message data in bytes

    uint8_t ip_addr_[4];
    uint8_t netmask_[4];
    uint8_t gateway_[4];
    uint8_t dns_[4];
    uint8_t mac_addr_[6];

    Socket_t xUDPSocket;

    std::atomic<bool> svr_ip_addr_lock_ {false};
    uint8_t svr_ip_addr_[4];

    int32_t udp_socket_;


    void read_network_config( const std::string& filename );
    bool string_to_addr( const std::string& addr_str, uint8_t* addr );
    
    virtual void udp_rx_task();
    virtual void udp_tx_task();

    virtual void rx_msg_proc();
    virtual void tx_msg_proc() = 0;    

public:
    explicit Network( ZynqDetector* owner, int udp_port );
    void network_init();
    void network_task_init();
};
 
