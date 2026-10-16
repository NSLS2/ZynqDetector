
#include <stdio.h>
#include <string>

#include "I2cBus.hpp"

I2cBus::I2cBus( uint8_t       bus_index
              , std::string   name
              , QueueHandle_t req_queue
              , QueueHandle_t resp_queue
              );
              : bus_index_  ( bus_index  )
              , name_       ( name       )
              , req_queue_  ( req_queue  )
              , resp_queue_ ( resp_queue )
{}


static void I2cBus::create_i2cbus_task()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { task(); });
    xTaskCreateStatic( task_wrapper, name_.c_str(), TASK_STACK_SIZE, &task_func, 1, task_stack, &task_tcb );
}
