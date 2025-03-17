#pragma once

#include <string>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "xiicps.h"
#include "xparameters.h"

#define I2C0_DEVICE_ID XPAR_XIICPS_0_DEVICE_ID // or use the base address if no device ID
#define I2C1_DEVICE_ID XPAR_XIICPS_1_DEVICE_ID // or use the base address if no device ID

// Define I2C base addresses from device tree
#define I2C0_BASE_ADDRESS 0xe0004000
#define I2C1_BASE_ADDRESS 0xe0005000

// Define I2C clock frequency from device tree
#define I2C_CLOCK_FREQUENCY 100000 // 0x61a80 in hex is 400000; 100000 is 100KHz

class PSI2C
{
private:
    XIicPs_Config *i2cps_config_ptr_;
    XIicPs i2c_ps_;

    uint8_t  bus_index_;
    std::string name_;
    uint32_t base_address_;

    SemaphoreHandle_t mutex_;

    int write( char* buffer, uint16_t length, uint16_t slave_address );
    int read( char* buffer, uint16_t length, uint16_t slave_address );

    void task();

public:

    struct PSI2CReq
    {
        uint16_t op;
        uint8_t  length;
        uint8_t  addr;
        uint8_t  data[4];
    };

    struct PSI2CResp
    {
        uint16_t op;
        uint8_t  length;
        uint8_t  data[4];
    };
    
    PSI2C( uint8_t bus_index, std::string name, QueueHandle_t req_queue, QueueHandle_t resp_queue );

    void create_psi2c_task();
};
