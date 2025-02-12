#pragma once

#include "zynqDetector.hpp"
#include <cstdint>
#include <array>


typedef uint16_t msg_id_t;



// 2-bit message op field
const uint8_t MSG_OP_WR = 0;  // write a register/device
const uint8_t MSG_OP_RD = 1;  // read a register/device
const uint8_t MSG_OP_SET_POLL_PERIOD = 2; // set polling period
const uint8_t MSG_OP_ADD_TO_POLL = 3;     // add a register/device to polling list


class Network
{
protected:
    ZynqDetector& detector;
    


public:
    Network( ZynqDetector& detector ) : detector( detector ){}

    void network_init();


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