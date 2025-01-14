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

typedef uint32_t reg_addr_t;

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
typedef struct {
    uint16_t  preamble;
    uint16_t  msg_id;
    uint8_t   msg_leng;
    uint8_t   op;
    uint32_t  addr;  // OP dependent, register address/I2C address/peripheral index/...
} udp_msg_t;

const std::vector<msg_code> msg_codec
    { msg_code { 0x0, 4, 0, 0 }
    , msg_code { 0x1, 4, 0, 4 }
    };

// Peripheral device definitions
typedef uint8_t interface_type_t;
const interface_type_t I2C = 0;
const interface_type_t SPI = 1;

typedef uint8_t device_t;
const device_t AD5254 = 0;
const device_t AD5593 = 1;
const device_t AD9249 = 2;

typedef struct
{
    device_t  device;
    uint8_t   addr;
} device_descriptor_t;

// device interface descriptor. Used for interface access in slow_access_task
typedef struct
{
    uint8_t if_type; // interface type: I2C, SPI, etc
    uint8_t if_no;   // interface number: 0, 1, ... to be used in combination with interf_type
    uint8_t if_data_reg;   // data register of the interface
    uint8_t if_instr_reg;  // instruction register for the interface
    std::vector<device_descriptor_t> dev_desc_v {};  // vector of devices connected to the interface
} interface_descriptor_t;

std::vector<device_descriptor_t> dev_desc_v{};

std::vector<interface_descriptor_t> if_desc_v{};

void device_registration( std::vector<device_interface_t> dev_if_v& )
{
    dev_if_v.push_back({ IF_TYPE_I2C, 0, {AD5254, AD}})
}


class ZynqDetector
{
private:

    axi_reg reg;

    // Data types
    typedef struct
    {
        uint8_t  op;
        uint32_t addr;
    } fast_access_req_t;

    typedef struct
    {
        uint8_t  op;
        uint8_t  device;
        uint16_t addr;
    } slow_access_req_t;

    typedef struct
    {
        uint32_t op;
        uint32_t leng;
        uint32_t cfg_data[4096/4 - 1];
    } bulk_access_req_t;

    typedef struct
    {
        uint8_t  op;
        uint32_t addr;
    } fast_access_resp_t;

    typedef struct
    {
        uint8_t  op;
        uint8_t  device;
        uint16_t addr;
    } slow_access_resp_t;

    typedef struct
    {
        uint32_t op;
        uint32_t leng;
        uint32_t cfg_data[4096/4 - 1];
    } bulk_access_resp_t;

    // Tasks
    static void udp_rx_task( void *pvParameters );
    static void udp_tx_task( void *pvParameters );
    
    static void fast_access_task( void *pvParameters );  // access registers directly (fast)
    static void slow_access_task( void *pvParameters );  // access asic/peripherals for small amount of data (slow)
    static void bulk_access_task( void *pvParameters );  // access asic/peripheral for bulk data (slowest)

    // msg_id parsers
    void access_mode_decode( msg_id_t msg_id );
    reg_addr_t fast_access_parse( msg_id_t msg_id );
    

    // Task handlers
    TaskHandle_t  udp_rx_task_handle;
    TaskHandle_t  udp_tx_task_handle;
    TaskHandle_t  fast_access_task_handle;
    TaskHandle_t  slow_access_task_handle;

    // Queues
    const uint16_t FAST_ACCESS_REQ_QUEUE_LENG  = 100;
    const uint16_t SLOW_ACCESS_REQ_QUEUE_LENG  = 100;
    const uint16_t BULK_ACCESS_REQ_QUEUE_LENG  = 4;
    const uint16_t FAST_ACCESS_RESP_QUEUE_LENG = 100;
    const uint16_t SLOW_ACCESS_RESP_QUEUE_LENG = 100;
    const uint16_t BULK_ACCESS_RESP_QUEUE_LENG = 4;

    const uint16_t FAST_ACCESS_REQ_QUEUE_SIZE  = FAST_ACCESS_REQ_QUEUE_LENG  * sizeof(fast_access_req_t);
    const uint16_t SLOW_ACCESS_REQ_QUEUE_SIZE  = SLOW_ACCESS_REQ_QUEUE_LENG  * sizeof(slow_access_req_t);
    const uint16_t BULK_ACCESS_REQ_QUEUE_SIZE  = SLOW_ACCESS_REQ_QUEUE_LENG  * sizeof(bulk_access_req_t);
    const uint16_t FAST_ACCESS_RESP_QUEUE_SIZE = FAST_ACCESS_RESP_QUEUE_LENG * sizeof(fast_access_resp_t);
    const uint16_t SLOW_ACCESS_RESP_QUEUE_SIZE = SLOW_ACCESS_RESP_QUEUE_LENG * sizeof(slow_access_resp_t);
    const uint16_t BULK_ACCESS_RESP_QUEUE_SIZE = SLOW_ACCESS_RESP_QUEUE_LENG * sizeof(bulk_access_resp_t);

    QueueHandle_t fast_access_req_queue  = NULL;
    QueueHandle_t slow_access_req_queue  = NULL;
    QueueHandle_t bulk_access_req_queue  = NULL;
    QueueHandle_t fast_access_resp_queue = NULL;
    QueueHandle_t slow_access_resp_queue = NULL;
    QueueHandle_t bulk_access_resp_queue = NULL;

    QueueSetHandle_t slow_req_queue_set;
    QueueSetHandle_t resp_queue_set;
    
    QueueSetMemberHandle_t active_slow_req_queue;
    QueueSetMemberHandle_t active_resp_queue;

    TimerHandle_t xPollTimer = NULL;

    std::vector<uint32_t> poll_list{};  // PVs to be polled


    // Parameters to be passed to the tasks
    typedef struct
    {
        QueueHandle_t fast_access_req_queue;
        QueueHandle_t slow_access_req_queue;
        QueueHandle_t bulk_access_req_queue;
    } udp_rx_task_ingress_param;

    typedef struct 
    {
        QueueHandle_t fast_access_req_queue;
        QueueHandle_t fast_access_resp_queue;
    } fast_access_task_ingress_param;

    typedef struct
    {
        QueueHandle_t    slow_access_req_queue;
        QueueHandle_t    bulk_access_req_queue;
        QueueSetHandle_t slow_req_queue_set;
        QueueHandle_t    slow_access_resp_queue;
        QueueHandle_t    bulk_access_resp_queue;        
    } slow_access_task_ingress_param;

    typedef struct
    {
        QueueHandle_t    fast_access_resp_queue;
        QueueHandle_t    slow_access_resp_queue;
        QueueHandle_t    bulk_access_resp_queue;
        QueueSetHandle_t resp_queue_set;
    } udp_tx_task_ingress_param;

public:

    ZynqDetector();
    ~ZynqDetector();

};
