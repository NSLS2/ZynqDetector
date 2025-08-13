/**
 * @file GermaniumDetector.cpp
 * @brief Member function definitions of `GermaniumDetector`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include <cstdint>
#include <functional>

#include "FreeRTOS.h"
#include "task.h"
#include "xil_printf.h"

#include "task_wrap.hpp"

#include "Register.hpp"

//===========================================================================//

/**
 * @brief Register constructor.
 */
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

//===========================================================================//

/**
 * @brief Register single write.
 */
void Register::write( uint16_t offset, uint32_t value )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        *(volatile uint32_t*)(base_addr_ + offset/4) = value;
        xSemaphoreGive( mutex_ );
    }
}
    
//===========================================================================//

/**
 * @brief Register single read.
 */
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

//===========================================================================//

/**
 * @brief Register single read.
 */
void Register::set_status( uint32_t status )
{
    write( 0xC, status );
}

//===========================================================================//

/**
 * @brief Start register multiple access.
 */
void Register::multi_access_start()
{
    while( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdFALSE );
}

//===========================================================================//

/**
 * @brief Register multiple write.
 */
void Register::multi_access_write( uint16_t offset, uint32_t value )
{
    *(volatile uint32_t*)(base_addr_ + offset/4) = value;
}
    
//===========================================================================//

/**
 * @brief Register multiple read.
 */
uint32_t Register::multi_access_read( uint16_t offset )
{
    return *(volatile uint32_t*)(base_addr_ + offset/4);
}

//===========================================================================//

/**
 * @brief End of register multiple access.
 */
void Register::multi_access_end()
{
    xSemaphoreGive( mutex_ );
}

//===========================================================================//

/**
 * @brief Register single access task function.
 */
void Register::single_access_task()
{
    RegisterSingleAccessReq  req;
    RegisterSingleAccessResp resp;

    uint16_t offset;

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

//===========================================================================//

/**
 * @brief Create register single access task.
 */

void Register::create_register_single_access_task()
{
    task_cfg_ = { .entry = [](void* ctx) { static_cast<Register*>(ctx)->single_access_task(); },
                  .context = this
                };

    xTaskCreateStatic( task_wrapper
                     , "Register Single Access"
                     , TASK_STACK_SIZE
                     , &task_cfg_
                     , TASK_PRIORITY
                     , task_stack_
                     , &task_tcb_ );
}

//===========================================================================//

/**
 * @brief Create register access tasks.
 */
void Register::create_register_access_tasks()
{
    create_register_single_access_task();
}

//===========================================================================//
