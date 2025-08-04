#pragma once

#include <string>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "xiicps.h"
#include "xparameters.h"

#include "Logger.hpp"

#include "queue.hpp"

//struct PsI2cAccessReqStruct
//{
//    uint16_t op;
//    uint8_t  length;
//    uint8_t  addr;
//    uint8_t  read;
//    uint8_t  data[4];
//};
//using PsI2cAccessReq = PsI2cAccessReqStruct;
//
//
//struct PsI2cAccessRespStruct
//{
//    uint16_t op;
//    uint8_t  length;
//    uint8_t  data[4];
//};
//using PsI2cAccessResp = PsI2cAccessRespStruct;



class PsI2c
{
private:

    //decltype(XPAR_XIICPS_0_DEVICE_ID) I2C0_DEVICE_ID XPAR_XIICPS_0_DEVICE_ID; // or use the base address if no device ID
    //decltype(XPAR_XIICPS_1_DEVICE_ID) I2C1_DEVICE_ID XPAR_XIICPS_1_DEVICE_ID; // or use the base address if no device ID

    //// Define I2C base addresses from device tree
    //decltype(0xe0004000) I2C0_BASE_ADDRESS 0xe0004000;
    //decltype(0xe0005000) I2C1_BASE_ADDRESS 0xe0005000;

    //// Define I2C clock frequency.
    //// 0x61a80 in hex is 400000; 100000 is 100KHz.
    //decltype(100000) I2C_CLOCK_FREQUENCY 100000

    XIicPs_Config* i2cps_config_ptr_;
    XIicPs         i2c_ps_;

    uint8_t        bus_index_;
    std::string    name_;
    uint32_t       device_id_;
    uintptr_t      base_addr_;
    uint32_t       clk_freq_;
    QueueHandle_t  req_queue_;
    QueueHandle_t  resp_queue_;
    
    xSemaphoreHandle mutex_;

    const Logger& logger_;

    int write( char* buffer, uint16_t length, uint16_t slave_address );
    int read( char* buffer, uint16_t length, uint16_t slave_address );

    void task();

public:
  
    PsI2c( const uint8_t                  bus_index
         , const std::string              name
         //, const uint32_t                 device_id
         , const uint32_t                 base_addr
         , const uint32_t                 clk_freq
         , const QueueHandle_t            req_queue
         , const QueueHandle_t            resp_queue
         , const Logger& logger
         );

    void create_psi2c_task();
};

