#pragma once

// C++ includes.
#include <vector>
#include <cstdint>
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

// Operations
const uint8_t REG_WR  { 0 };
const uint8_t REG_RD  { 1 };
const uint8_t SPI_WR  { 2 };
const uint8_t SPI_RD  { 3 };
const uint8_t I2C_WR  { 4 };
const uint8_t I2C_RD  { 5 };
const uint8_t ASIC_WR { 6 };
const uint8_t ASIC_RD { 7 };

// Message coding
struct msg_code {
    uint_fast16_t  msg_id;
    uint_fast8_t   msg_leng;
    uint_fast8_t   op;
    uint_fast32_t  addr;  // OP dependent, register address/I2C address/peripheral index/...
};

const std::vector<msg_code> msg_codec
    { msg_code { 0x0, 4, 0, 0 }
    , msg_code { 0x1, 4, 0, 4 }
    };

typedef struct {
    uint8_t  op;
    uint32_t addr;
} fast_access_req_t;

typedef struct {
    uint8_t  op;
    uint8_t  device;
    uint16_t addr;
} slow_access_req_t;

typedef struct {
    uint32_t op;
    uint32_t leng;
    uint32_t cfg_data[4096/4 - 1];
} asic_cfg_req_t;

typedef struct {
    uint8_t  op;
    uint32_t addr;
} fast_access_resp_t;

typedef struct {
    uint8_t  op;
    uint8_t  device;
    uint16_t addr;
} slow_access_resp_t;

typedef struct {
    uint32_t op;
    uint32_t leng;
    uint32_t cfg_data[4096/4 - 1];
} asic_cfg_resp_t;


class ZynqDetector {
private:
    axi_reg reg;

    // Tasks
    void udp_rx_task( void *pvParameters );
    void udp_tx_task( void *pvParameters );
    void fast_access_task( void *pvParameters );  // access registers directly (fast)
    void slow_access_task( void *pvParameters );  // access peripherals (slow)
    void asic_cfg_task( void *pvParameters );


    // Task handlers
    TaskHandle_t  udp_rx_task_handle;
    TaskHandle_t  udp_tx_task_handle;
    TaskHandle_t  fast_access_task_handle;
    TaskHandle_t  slow_access_task_handle;

    // Queues
    const uint16_t FAST_ACCESS_REQ_QUEUE_LENG  = 100;
    const uint16_t SLOW_ACCESS_REQ_QUEUE_LENG  = 100;
    const uint16_t ASIC_CFG_REQ_QUEUE_LENG     = 4;
    const uint16_t FAST_ACCESS_RESP_QUEUE_LENG = 100;
    const uint16_t SLOW_ACCESS_RESP_QUEUE_LENG = 100;
    const uint16_t ASIC_CFG_RESP_QUEUE_LENG    = 4;

    const uint16_t FAST_ACCESS_REQ_QUEUE_SIZE  = FAST_ACCESS_REQ_QUEUE_LENG  * sizeof(fast_access_req_t);
    const uint16_t SLOW_ACCESS_REQ_QUEUE_SIZE  = SLOW_ACCESS_REQ_QUEUE_LENG  * sizeof(slow_access_req_t);
    const uint16_t ASIC_CFG_REQ_QUEUE_SIZE     = SLOW_ACCESS_REQ_QUEUE_LENG  * sizeof(asic_cfg_req_t);
    const uint16_t FAST_ACCESS_RESP_QUEUE_SIZE = FAST_ACCESS_RESP_QUEUE_LENG * sizeof(fast_access_resp_t);
    const uint16_t SLOW_ACCESS_RESP_QUEUE_SIZE = SLOW_ACCESS_RESP_QUEUE_LENG * sizeof(slow_access_resp_t);
    const uint16_t ASIC_CFG_RESP_QUEUE_SIZE    = SLOW_ACCESS_RESP_QUEUE_LENG * sizeof(asic_cfg_resp_t);

    QueueHandle_t fast_access_req_queue  = NULL;
    QueueHandle_t slow_access_req_queue  = NULL;
    QueueHandle_t asic_cfg_req_queue     = NULL;
    QueueHandle_t fast_access_resp_queue = NULL;
    QueueHandle_t slow_access_resp_queue = NULL;
    QueueHandle_t asic_cfg_resp_queue    = NULL;

    QueueSetHandle_t slow_req_queue_set;
    QueueSetHandle_t resp_queue_set;
    
    QueueSetMemberHandle_t active_slow_req_queue;
    QueueSetMemberHandle_t active_resp_queue;



    TimerHandle_t xPollTimer = NULL;

    std::vector<uint32_t> poll_list{};  // PVs to be polled

public:

    ZynqDetector();
    ~ZynqDetector();

    void udp_rx();
    void udp_tx();


}
