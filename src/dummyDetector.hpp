#pragma once

// This is an example of detector class derived from ZynqDetector.

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

const uint16_t UDP_MSG_ID = 0x5053;
// Message coding
typedef struct {
    uint16_t  id;
    uint16_t  op;
    uint32_t  payload[ MAX_UDP_MSG_LENG - 1 ];
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


#define NORMAL               0x0
#define NETWORK_INIT_FAILURE 0x1
#define NETWORK_TX_FAILURE   0x2
#define NETWORK_RX_FAILURE   0x3


typedef struct
{
    device_t  device;
    uint8_t   addr;
} device_descriptor_t;

// device interface descriptor. Used for interface access in i2c0_access_task
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


class dummyDetector : public ZynqDetector {
protected:
    //======================================
    // Data types
    //======================================
    typedef struct
    {
        uint8_t  op;
        uint32_t addr;
        uint32_t data;
    } reg_access_req_t;

    typedef struct
    {
        uint8_t  op;
        uint8_t  device;
        uint16_t addr;
    } i2c_access_req_t;


    // Parameters to be passed to the tasks
    typedef struct
    {
        QueueHandle_t reg_access_req_queue;
        QueueHandle_t i2c0_access_req_queue;
        QueueHandle_t bulk_access_req_queue;
    } udp_rx_task_ingress_param;

    typedef struct 
    {
        QueueHandle_t reg_access_req_queue;
        QueueHandle_t reg_access_resp_queue;
    } reg_access_task_ingress_param;

    typedef struct
    {
        QueueHandle_t    i2c0_access_req_queue;
        QueueHandle_t    bulk_access_req_queue;
        QueueSetHandle_t slow_req_queue_set;
        QueueHandle_t    i2c0_access_resp_queue;
        QueueHandle_t    bulk_access_resp_queue;        
    } i2c0_access_task_ingress_param;

    typedef struct
    {
        QueueHandle_t    reg_access_resp_queue;
        QueueHandle_t    i2c0_access_resp_queue;
        QueueHandle_t    bulk_access_resp_queue;
        QueueSetHandle_t resp_queue_set;
    } udp_tx_task_ingress_param;


    //======================================
    // Queues
    //======================================
    const uint16_t REG_ACCESS_REQ_QUEUE_LENG  = 100;
    const uint16_t I2C0_ACCESS_REQ_QUEUE_LENG = 100;

    const uint16_t REG_ACCESS_REQ_QUEUE_SIZE  = REG_ACCESS_REQ_QUEUE_LENG  * sizeof(reg_access_req_t);
    const uint16_t I2C0_ACCESS_REQ_QUEUE_SIZE = I2C0_ACCESS_REQ_QUEUE_LENG  * sizeof(i2c_access_req_t);

    QueueHandle_t reg_access_req_queue  = NULL;
    QueueHandle_t i2c0_access_req_queue  = NULL;
    QueueHandle_t bulk_access_req_queue  = NULL;
    QueueHandle_t i2c0_access_resp_queue = NULL;
    QueueHandle_t bulk_access_resp_queue = NULL;

    QueueSetHandle_t slow_req_queue_set;
    QueueSetHandle_t resp_queue_set;

    //======================================
    // Tasks
    //======================================
    static void reg_access_task( void *pvParameters );  // access registers directly (fast)
    static void i2c0_access_task( void *pvParameters );  // access asic/peripherals for small amount of data (slow)
    static void bulk_access_task( void *pvParameters );  // access asic/peripheral for bulk data (slowest)

    TaskHandle_t  reg_access_task_handle;
    TaskHandle_t  i2c0_access_task_handle;

    //======================================
    // Access request processing
    //======================================
    virtual void reg_access_req_proc( const reg_access_req_t& reg_access_req ) = 0; // fast access request process
    virtual void i2c0_access_req_proc( const reg_access_req_t& reg_access_req ) = 0; // slow access request process
    virtual void bulk_access_req_proc( const reg_access_req_t& reg_access_req ) = 0; // bulk access request process

    //======================================
    // ISR
    //======================================
    void i2c0_isr()
    {
        
    }



};