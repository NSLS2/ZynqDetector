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



class ZynqDetector : public FPGA
{
protected:

    //==================================================
    //                   Data types                   //
    //==================================================

    //------------------------------
    // UDP message
    //------------------------------
    const int MAX_UDP_MSG_DATA_LENG = 
    typedef struct {
        uint16_t id;
        uint16_t op;
        uint32_t data[];
    } udp_rx_msg_t;

    typedef struct {
        uint16_t id;
        uint16_t op;
        uint32_t data[];
    } udp_rx_msg_t;

    //------------------------------
    // Access request
    //------------------------------
    typedef struct
    {
        uint8_t  op;
        bool     read;
        uint32_t addr;
        uint32_t data;
    } reg_access_req_t;

    typedef struct
    {
        uint8_t  op;
        bool     read;
        uint8_t  device_addr;
        uint16_t instr_reg_addr;
        uint16_t data_reg_addr;
        uint32_t data;
    } interface_single_access_req_t;

    typedef struct
    {
        uint32_t op;
        bool     read;
        uint32_t leng;
        uint8_t  device_addr;
        uint16_t instr_reg_addr;
        uint16_t data_reg_addr;
        uint32_t data[4096/4 - 1];
    } interface_multi_access_req_t;

    //------------------------------
    // Access response
    //------------------------------
    typedef struct
    {
        uint8_t  op;
        uint32_t addr;
    } reg_access_resp_t;

    typedef struct
    {
        uint8_t   op;
        uint32_t  data;
    } interface_single_access_resp_t;
    
    typedef struct
    {
        uint8_t  op;
        uint32_t leng;
        uint32_t data[4096/4 - 1];
    } interface_multi_access_resp_t;

    //------------------------------
    // Task parameter
    //------------------------------
    typedef struct
    {
        QueueHandle_t* reg_access_req_queue;
        QueueHandle_t* interface_single_access_req_queue;
        QueueHandle_t* interface_multi_access_req_queue;
    } udp_rx_task_param_t;

    typedef struct
    {
        QueueHandle_t* req_queue;
        QueueHandle_t* resp_queue;
    } reg_access_task_param_t;

    typedef struct
    {
        QueueHandle_t* req_queue;
        QueueHandle_t* resp_queue;
        void* read();
        void* write();
    } interface_single_access_task_param_t;

    typedef struct
    {
        QueueHandle_t* req_queue;
        QueueHandle_t* resp_queue;
        void* read();
        void* write();
    } interface_multi_access_task_param_t;


    //==================================================
    //                    Variables                   //
    //==================================================
    uint32_t base_addr_;

    //------------------------------
    // Interrupt
    //------------------------------
    XScuGic gic;                               // Global Interrupt Controller
    std::map<int, TaskHandle_t> irq_task_map;  // IRQ task map as an instance member

    //------------------------------
    // Queues
    //------------------------------
    const size_t REG_ACCESS_REQ_QUEUE_LENG  = 100;
    const size_t REG_ACCESS_RESP_QUEUE_SIZE = REG_ACCESS_RESP_QUEUE_LENG * sizeof(reg_access_resp_t);
    const size_t REG_ACCESS_RESP_QUEUE_LENG = 100;
    const size_t REG_ACCESS_RESP_QUEUE_SIZE = REG_ACCESS_RESP_QUEUE_LENG * sizeof(reg_access_resp_t);

    const size_t INTERFACE_SINGLE_ACCESS_REQ_QUEUE_LENG  = 10;
    const size_t INTERFACE_SINGLE_ACCESS_RESP_QUEUE_SIZE = INTERFACE_SINGLE_ACCESS_RESP_QUEUE_LENG * sizeof(interface_single_access_resp_t);
    const size_t INTERFACE_SINGLE_ACCESS_RESP_QUEUE_LENG = 10;
    const size_t INTERFACE_SINGLE_ACCESS_RESP_QUEUE_SIZE = INTERFACE_SINGLE_ACCESS_RESP_QUEUE_LENG * sizeof(interface_single_access_resp_t);

    const size_t INTERFACE_MULTI_ACCESS_REQ_QUEUE_LENG  = 5;
    const size_t INTERFACE_MULTI_ACCESS_RESP_QUEUE_SIZE = INTERFACE_MULTI_ACCESS_RESP_QUEUE_LENG * sizeof(interface_multi_access_resp_t);
    const size_t INTERFACE_MULTI_ACCESS_RESP_QUEUE_LENG = 5;
    const size_t INTERFACE_MULTI_ACCESS_RESP_QUEUE_SIZE = INTERFACE_MULTI_ACCESS_RESP_QUEUE_LENG * sizeof(interface_multi_access_resp_t);

    QueueSetMemberHandle_t active_resp_queue;
    QueueSetHandle_t resp_queue_set;

    
    //------------------------------
    // Task handlers
    //------------------------------
    TaskHandle_t  udp_rx_task_handle;
    TaskHandle_t  udp_tx_task_handle;
    TaskHandle_t  fast_access_task_handle;
    TaskHandle_t  slow_access_task_handle;
        
    QueueHandle_t reg_access_req_queue               = NULL;
    QueueHandle_t reg_access_req_queue               = NULL;
    QueueHandle_t interface_single_access_req_queue  = NULL;
    QueueHandle_t interface_single_access_resp_queue = NULL;
    QueueHandle_t interface_multi_access_req_queue   = NULL;
    QueueHandle_t interface_multi_access_resp_queue  = NULL;

    
    //------------------------------
    // Network
    //------------------------------

    const uint16_t MAX_UDP_MSG_LENG = 4096;
    const uint16_t MAX_UDP_MSG_DATA_LENG = MAX_UDP_MSG_LENG - 4; // length of message data in bytes

    uint8_t ip_addr[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint8_t dns[4];
    uint8_t mac_addr[6];
    const uint32_t UDP_PORT = 25913;

    std::atomic<bool> svr_ip_addr_lock {false};
    uint8_t svr_ip_addr[4];

    int32_t udp_socket;

    TimerHandle_t xPollTimer = NULL;

    std::vector<uint32_t> poll_list{};  // PVs to be polled


    //==================================================
    //                    Functions                   //
    //==================================================
    //------------------------------
    // Interrupt
    //------------------------------
    virtual static void ISR_Wrapper(void* context) = 0;

    //------------------------------
    // Network
    //------------------------------
    bool string_to_addr( const std::string& addr_str, uint8_t* addr );
    void read_network_config( const std::string& filename );

    virtual void udp_rx_task( void *pvParameters );
    virtual void udp_tx_task( void *pvParameters );

    // Set failure number to register.
    void set_fail_num( uint32_t fail_num );

    template <typename T>
    void ZynqDetector::report_err( const std::string& s, T err_code, uint32_t fail_num );
    //==================================================

public:

    ZynqDetector( uint32_t base_addr );
    ~ZynqDetector();

    virtual void isr_handler() = 0;
    virtual void register_isr() = 0;
    void DummyDetector::create_irq_task_map();

    void network_init();
    virtual void queue_init();
    virtual void task_init();

    template <typename T>
    void ZynqDetector::report_error( const std::string& s, T err_code, uint32_t fail_num );

};
