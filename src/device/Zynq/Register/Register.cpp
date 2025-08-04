#include <cstdint>
#include <functional>

#include "FreeRTOS.h"
#include "xil_printf.h"

#include "task_wrap.hpp"

#include "Register.hpp"

//=========================================
// Register class
//=========================================
Register::Register
    (
      uintptr_t            base_addr
    , const QueueHandle_t  single_access_req_queue
    , const QueueHandle_t  single_access_resp_queue
    , const Logger&        logger
    )
    : base_addr_ ( static_cast<uintptr_t>( base_addr )              )
    , single_access_req_queue_           ( single_access_req_queue  )
    , single_access_resp_queue_          ( single_access_resp_queue )
    , logger_                            ( logger                   )
{
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == NULL)
    {
        xil_printf("Failed to initialize register mutex!\n");
    }
}

//=========================================
// Register single access
//=========================================
void Register::write( uint16_t offset, uint32_t value )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        *(volatile uint32_t*)(base_addr_ + offset/4) = value;
        xSemaphoreGive( mutex_ );
    }
}
    
uint32_t Register::read( uint16_t offset )
{
    uint32_t value = 0;
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        value = *(volatile uint32_t*)(base_addr_ + offset/4);
        xSemaphoreGive( mutex_ );
    }
    return value;
}

void Register::set_status( uint32_t status )
{
    write( 0xC, status );
}

//=========================================
// Register multiple access
//=========================================
void Register::multi_access_start()
{
    while( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdFALSE );
}

void Register::multi_access_write( uint16_t offset, uint32_t value )
{
    *(volatile uint32_t*)(base_addr_ + offset/4) = value;
}
    
uint32_t Register::multi_access_read( uint16_t offset )
{
    return *(volatile uint32_t*)(base_addr_ + offset/4);
}

void Register::multi_access_end()
{
    xSemaphoreGive( mutex_ );
}


void Register::single_access_task()
{
    RegisterSingleAccessReq  req;
    RegisterSingleAccessResp resp;

    uint16_t offset;

    //auto param = static_cast<reg_single_access_task_param_t*>(pvParameters);

    while(1)
    {
        xQueueReceive( single_access_req_queue_
                     , &req
					 , portMAX_DELAY );
        
        offset = req.op & 0x7fff;

        if ( (req.op & 0x8000) != 0 )
        {
            resp.data = read( offset );
            resp.op = req.op;
            xQueueSend( single_access_resp_queue_
                      , &resp
                      , 0UL
                      );
        }
        else
        {
            write( req.data, offset );
        }
    }
}


void Register::create_register_single_access_task()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { single_access_task(); });
    xTaskCreate( task_wrapper, "Register Single Access", 1000, &task_func, 1, NULL );
}


void Register::create_register_access_tasks()
{
    create_register_single_access_task();
}

//=========================================
