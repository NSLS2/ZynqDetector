#pragma once

// C++ includes.
#include <vector>
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

#include "fpga.hpp"
#include <cstdint>

// Message coding
typedef struct {
    uint_fast16_t  msg_id;
    uint_fast8_t   msg_leng;
    uint_fast8_t   op;
    uint_fast32_t  reg_addr;
} msg_code_t;

const std::vector<msg_code_t> msg_codec {
    msg_code_t 
}


class ZynqDetector {
private:
    axi_reg reg;

    // Tasks
    static void udp_rx_task( void *pvParameters );
    static void udp_tx_task( void *pvParameters );
    static void fast_access_task( void *pvParameters );  // access registers directly (fast)
    static void slow_access_task( void *pvParameters );  // access peripherals (slow)

    // Task handlers
    TaskHandle_t  udp_rx_task_handle;
    TaskHandle_t  udp_tx_task_handle;
    TaskHandle_t  fast_access_task_handle;
    TaskHandle_t  slow_access_task_handle;

    // Queues
    QueueHandle_t fast_access_req_queue = NULL;
    QueueHandle_t slow_access_req_queue = NULL;
    QueueHandle_t fast_access_resp_queue = NULL;
    QueueHandle_t slow_access_resp_queue = NULL;
    TimerHandle_t xPollTimer = NULL;

public:

    ZynqDetector();
    ~ZynqDetector();

    void udp_rx();
    void udp_tx();


}
