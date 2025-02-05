#pragma once

// C++ includes.
#include <vector>
#include <cstdint>
#include <memory>
// FreeRTOS includes.
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "FreeRTOS_IP.h"
#include "network_interface.h"
#include "ff.h"  // FatFS file system library for SD card
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"

#include "msg.hpp"
#include "fpga.hpp"
#include "network.hpp"



class ZynqDetector
{
protected:

    // Data types
    typedef struct
    {
        uint8_t  op;
        uint32_t addr;
    } fast_access_resp_t;

    // Queues
    const uint16_t FAST_ACCESS_RESP_QUEUE_LENG = 100;
    const uint16_t FAST_ACCESS_RESP_QUEUE_SIZE = FAST_ACCESS_RESP_QUEUE_LENG * sizeof(fast_access_resp_t);

    axi_reg reg;
    std::unique_ptr<Network> net;

    // Tasks
    static void udp_rx_task( void *pvParameters );
    static void udp_tx_task( void *pvParameters );
    // Task handlers
    TaskHandle_t  udp_rx_task_handle;
    TaskHandle_t  udp_tx_task_handle;
    
    // msg_id parsers
    void access_mode_decode( msg_id_t msg_id );
    reg_addr_t fast_access_parse( msg_id_t msg_id );
    
    QueueHandle_t fast_access_resp_queue = NULL;

    
    QueueSetMemberHandle_t active_slow_req_queue;
    QueueSetMemberHandle_t active_resp_queue;

    TimerHandle_t xPollTimer = NULL;

    std::vector<uint32_t> poll_list{};  // PVs to be polled

    virtual void queue_init();
    virtual void create_tasks();

    // Network related
    bool string_to_addr( const std::string& addr_str, uint8_t* addr );
    void read_network_config( const std::string& filename );
    void network_init();

public:

    ZynqDetector();
    ~ZynqDetector();

    // Set failure number to register.
    void set_fail_num( uint32_t fail_num );

    template <typename T>
    void ZynqDetector::report_error( const std::string& s, T err_code, uint32_t fail_num );

    void Network::init

};
