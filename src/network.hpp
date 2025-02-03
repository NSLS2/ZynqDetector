#pragma once

#include "zynqDetector.hpp"
#include <cstdint>
#include <array>


typedef uint16_t msg_id_t;

const uint16_t MAX_UDP_MSG_LENG = 4096;
const uint16_t MAX_UDP_MSG_DATA_LENG = MAX_UDP_MSG_LENG - 4; // length of message data in bytes

typedef struct {
    uint16_t id;
    uint16_t op;
    uint32_t msg_data[MAX_UDP_MSG_DATA_LENG];
} udp_msg_t;

// 2-bit message op field
const uint8_t MSG_OP_WR = 0;  // write a register/device
const uint8_t MSG_OP_RD = 1;  // read a register/device
const uint8_t MSG_OP_SET_POLL_PERIOD = 2; // set polling period
const uint8_t MSG_OP_ADD_TO_POLL = 3;     // add a register/device to polling list


class Network
{
private:
    ZynqDetector& detector;
    
    const uint32_t UDP_PORT = 25913;

    uint8_t ip_addr[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint8_t dns[4];
    uint8_t mac_addr[6];

    std::atomic<bool> svr_ip_addr_lock {false};
    uint8_t svr_ip_addr[4];

    int32_t udp_socket;

    void read_network_config( const std::string& filename );
    bool string_to_addr( const std::string& addr_str, uint8_t* addr );

public:
    Network( ZynqDetector& detector ) : detector( detector ){}

    void udp_rx_task( void *pvParameters );
    void udp_tx_task( void *pvParameters );

    void report_error( const std::string& s, T err_code, uint32_t fail_num )
    {
        detector.report_error( s, err_code, fail_num );
    }

    static void udp_rx_task_wrapper( void* pvParameters )
    {
        Network* instance = std::static_cast<Network*>( pvParameters );
        instance->udp_rx_task();
    }

    static void udp_tx_task_wrapper( void* pvParameters )
    {
        Network* instance = std::static_cast<Network*>( pvParameters );
        instance->udp_tx_task();
    }

    static std::unique_ptr<Network> createInstance();
    
    
    
};

typedef struct {
    uint32_t leng;
    uint32_t cfg_data[MAX_UDP_MSG_DATA_LENG / 4 - 1];
} asic_cfg_t;