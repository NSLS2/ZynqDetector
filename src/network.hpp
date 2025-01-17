#pragma once

#include <cstdint>
#include <array>


typedef uint16_t msg_id_t;

const uint16_t MAX_UDP_MSG_LENG = 4096;
const uint16_t MAX_UDP_MSG_DATA_LENG = MAX_UDP_MSG_LENG - 4; // length of message data in bytes

typedef struct {
    uint16_t hdr;
    uint16_t op;
    uint32_t msg_data[MAX_UDP_MSG_DATA_LENG];
} udp_msg_t;

// 2-bit message op field
const uint8_t MSG_OP_WR = 0;  // write a register/device
const uint8_t MSG_OP_RD = 1;  // read a register/device
const uint8_t MSG_OP_SET_POLL_PERIOD = 2; // set polling period
const uint8_t MSG_OP_ADD_TO_POLL = 3;     // add a register/device to polling list


class network
{
private:
    uint8_t ip_addr[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint8_t dns[4];
    uint8_t mac_addr[6];

    void read_network_config( const std::string& filename );

public:
    network();
    static void udp_rx( void *pvParameters );
    static void udp_tx( void *pvParameters );

    int32_t udp_socket;
};

typedef struct {
    uint32_t leng;
    uint32_t cfg_data[MAX_UDP_MSG_DATA_LENG / 4 - 1];
} asic_cfg_t;