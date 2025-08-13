/**
 * @file PsI2c.cpp
 * @brief Member function definitions of `PsI2c`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "xil_printf.h"

#include "task_wrap.hpp"
#include "queue.hpp"
#include "PsI2c.hpp"

//===========================================================================//

/**
 * @brief PsI2c constructor
 */
PsI2c::PsI2c
    ( const uint8_t       bus_index
    , const std::string   name
    , const uint32_t      base_addr
    , const uint32_t      clk_freq
    , const QueueHandle_t req_queue
    , const QueueHandle_t resp_queue
    , const Logger& logger
    )
    : bus_index_  ( bus_index  )
    , name_       ( name       )
    , base_addr_  ( base_addr  )
    , clk_freq_   ( clk_freq   )
    , req_queue_  ( req_queue  )
    , resp_queue_ ( resp_queue )
    , logger_     ( logger     )
{
    ///< Initialize I2C
    i2cps_config_ptr_ = XIicPs_LookupConfig( base_addr_ );
    if ( i2cps_config_ptr_ == NULL )
    {
        xil_printf("I2C %d: lookup config failed\n", bus_index_ );
        return;
    }

    int status = XIicPs_CfgInitialize( &i2c_ps_, i2cps_config_ptr_, i2cps_config_ptr_->BaseAddress );
    if ( status != XST_SUCCESS )
    {
        xil_printf("I2C %d: config initialization failed\n", bus_index_ );
        return;
    }
    
    ///< Self Test
    status = XIicPs_SelfTest( &i2c_ps_ );
    if ( status != XST_SUCCESS )
    {
        xil_printf("I2C %d: self-test failed failed\n", bus_index_);
        return;
    }

    ///< Set clock frequency
    XIicPs_SetSClk( &i2c_ps_, clk_freq_ );
}

//===========================================================================//

/**
 * @brief Write to I2C bus.
 */
int PsI2c::write( char* buffer, uint16_t length, uint16_t slave_address )
{
    int status = XST_SUCCESS;

    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        status = XIicPs_MasterSendPolled( &i2c_ps_, (u8*)buffer, length, slave_address );
        xSemaphoreGive( mutex_ );

        if (status != XST_SUCCESS)
        {
            logger_.log_error("I2C %d: failed to send\n", bus_index_);
        }
    }

    return status;
}

//===========================================================================//

/**
 * @brief Read from I2C bus.
 */
int PsI2c::read( char* buffer, uint16_t length, uint16_t slave_address ) 
{
    int status = XST_SUCCESS;

    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        status = XIicPs_MasterRecvPolled( &i2c_ps_, (u8*)buffer, length, slave_address );
        xSemaphoreGive( mutex_ );
        
        if (status != XST_SUCCESS)
        {
            logger_.log_error("I2C %d: failed to receive\n", bus_index_);
        }
    }
    
    return status;
}

//===========================================================================//

/**
 * @brief Task function.
 */
void PsI2c::task()
{
    PsI2cAccessReq  req;
    PsI2cAccessResp resp;

    while(1)
    {
        xQueueReceive( req_queue_
                     , &req
					 , portMAX_DELAY
                     );
        
        if ( req.op && 0x8000 )
        {
            read( (char*)resp.data, req.length, req.addr );
            resp.op = req.op;
            xQueueSend( resp_queue_
                      , static_cast<const void*>(&resp)
                      , 0UL
                      );
        }
        else
        {
            write( (char*)req.data, req.length, req.addr );
        }
    }
}

//===========================================================================//

/**
 * @brief Creat PsI2c task.
 */
void PsI2c::create_psi2c_task()
{
    task_cfg_ = { .entry = [](void* ctx) { static_cast<PsI2c*>(ctx)->task(); }
                , .context = this
                };

    xTaskCreateStatic( task_wrapper
                     , name_.c_str()
                     , TASK_STACK_SIZE
                     , &task_cfg_
                     , 1
                     , task_stack_
                     , &task_tcb_
                     );
}

//===========================================================================//

