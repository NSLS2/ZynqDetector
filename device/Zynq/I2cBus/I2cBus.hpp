/**
 * @file I2c.hpp
 * @brief Class definition of `I2cBus`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <string>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "xiicps.h"
#include "xparameters.h"

#inlcude "queue.hpp"

//===========================================================================//

class I2cBus
{
protected:

    uint8_t           bus_index_;
    std::string       name_;

    xSemaphoreHandle  mutex_;

    static constexpr uint32_t TASK_STACK_SIZE = 1000;
    StaticTask_t              task_tcb_;
    StackType_t               task_stack_[TASK_STACK_SIZE];
    TaskConfig                task_cfg_;

    int write( char* buffer, uint16_t length, uint16_t slave_address );
    int read( char* buffer, uint16_t length, uint16_t slave_address );

    void task();

public:
    
    I2cBus( uint8_t bus_index
          , std::string name
          , QueueHandle_t req_queue
          , QueueHandle_t resp_queue );

    ~I2cBus() = default;

    void create_psi2c_task();
};

//===========================================================================//

