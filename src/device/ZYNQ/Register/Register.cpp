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
    RegisterReq  req;
    RegisterResp resp;

    char rd_data[4];
    char wr_data[4];

    auto param = static_cast<reg_access_task_param_t*>(pvParameters);

    while(1)
    {
        xQueueReceive( req_queue
                     , &req,
					 , portMAX_DELAY );
        
        if ( req.op == READ_TEMPERATURE )
        {
            read( &resp.data, req.length, req.addr );
            resp.op = req.op;
            xQueueSend( req_queue_
                      , resp,
                      , 0UL
                      );
        }
        else if ( req.op == READ_VCC )
        {
            read( &resp.data, req.length, req.addr );
            resp.op = req.op;
            xQueueSend( req_queue_
                      , resp,
                      , 0UL
                      )
        }
    }
}

static void PSXADC::task_wrapper(void* param, void (PSXADC::*task)())
{
    auto obj = statid_cast<PSXADC*>(param);
    if( obj )
    {
        obj->*task();
    }
    else
    {
        log_error("task_wrapper: Invalid cast\n");
    }
}
//=========================================
