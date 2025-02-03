#pragma once

class dummyDetector : public ZynqDetector {
protected:
    //======================================
    // Data types
    //======================================
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
        uint16_t leng;
        uint8_t  device;
        uint16_t addr;
    } slow_access_resp_t;

    typedef struct
    {
        uint32_t op;
        uint32_t leng;
        uint32_t cfg_data[4096/4 - 1];
    } bulk_access_resp_t;

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


    //======================================
    // Queues
    //======================================
    const uint16_t FAST_ACCESS_REQ_QUEUE_LENG  = 100;
    const uint16_t SLOW_ACCESS_REQ_QUEUE_LENG  = 100;
    const uint16_t BULK_ACCESS_REQ_QUEUE_LENG  = 4;
    const uint16_t SLOW_ACCESS_RESP_QUEUE_LENG = 100;
    const uint16_t BULK_ACCESS_RESP_QUEUE_LENG = 4;

    const uint16_t FAST_ACCESS_REQ_QUEUE_SIZE  = FAST_ACCESS_REQ_QUEUE_LENG  * sizeof(fast_access_req_t);
    const uint16_t SLOW_ACCESS_REQ_QUEUE_SIZE  = SLOW_ACCESS_REQ_QUEUE_LENG  * sizeof(slow_access_req_t);
    const uint16_t BULK_ACCESS_REQ_QUEUE_SIZE  = SLOW_ACCESS_REQ_QUEUE_LENG  * sizeof(bulk_access_req_t);
    const uint16_t SLOW_ACCESS_RESP_QUEUE_SIZE = SLOW_ACCESS_RESP_QUEUE_LENG * sizeof(slow_access_resp_t);
    const uint16_t BULK_ACCESS_RESP_QUEUE_SIZE = SLOW_ACCESS_RESP_QUEUE_LENG * sizeof(bulk_access_resp_t);

    QueueHandle_t fast_access_req_queue  = NULL;
    QueueHandle_t slow_access_req_queue  = NULL;
    QueueHandle_t bulk_access_req_queue  = NULL;
    QueueHandle_t slow_access_resp_queue = NULL;
    QueueHandle_t bulk_access_resp_queue = NULL;

    QueueSetHandle_t slow_req_queue_set;
    QueueSetHandle_t resp_queue_set;

    //======================================
    // Tasks
    //======================================
    static void fast_access_task( void *pvParameters );  // access registers directly (fast)
    static void slow_access_task( void *pvParameters );  // access asic/peripherals for small amount of data (slow)
    static void bulk_access_task( void *pvParameters );  // access asic/peripheral for bulk data (slowest)

    TaskHandle_t  fast_access_task_handle;
    TaskHandle_t  slow_access_task_handle;

    //======================================
    // Access request processing
    //======================================
    virtual void fast_access_req_proc( const fast_access_req_t& fast_access_req ) = 0; // fast access request process
    virtual void slow_access_req_proc( const fast_access_req_t& fast_access_req ) = 0; // slow access request process
    virtual void bulk_access_req_proc( const fast_access_req_t& fast_access_req ) = 0; // bulk access request process


};