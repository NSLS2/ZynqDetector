#include <cstdint>

#include "FreeRTOS.h"

#include "Register.hpp"



//=========================================
// Register class
//=========================================
Register::Register( uintptr_t base_addr )
{
    base_addr_ = reinterpret_cast<volatile uint32_t*>( base_addr );
}

void Register::write( uint32_t offset, uint32_t value )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        *(volatile uint32_t*)(base_addr_ + offset/4) = value;
        xSemaphoreGive( mutex_ );
    }
}
    
uint32_t Register::read( uint32_t offset )
{
    uint32_t value = 0;
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        value = *(volatile uint32_t*)(base_addr_ + offset/4);
        xSemaphoreGive( mutex_ );
    }
    return value;
}

void Register::task()
{
    RegisterAccessReq  req;
    RegisterAccessResp resp;

    uint32_t offset;

    auto param = static_cast<reg_access_task_param_t*>(pvParameters);

    while(1)
    {
        xQueueReceive( req_queue
                     , &req,
					 , portMAX_DELAY );
        
        offset = static_cast<uint32_t>( req.op & 0x7fff );

        if ( (req.op & 0x8000) != 0 )
        {
            resp.data = read( offset );
            resp.op = req.op;
            xQueueSend( resp_queue_
                      , resp,
                      , 0UL
                      );
        }
        else
        {
            write( req.data, offset );
        }
    }
}

void Register::create_register_task()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { task(); });
    xTaskCreate( task_wrapper, "Register Access", 1000, &task_func, 1, NULL );
}
//=========================================
