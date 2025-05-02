//#include <iostream>
#include <stdio.h>
#include <string>

#include "PSI2C.hpp"

PSI2C::PSI2C( const int8_t       bus_index
            , const std::string  name
            , const ueueHandle_t req_queue
            , const ueueHandle_t resp_queue );
    : bus_index_  ( bus_index  )
    , name_       ( name       )
    , req_queue_  ( req_queue  )
    , resp_queue_ ( resp_queue )
{
    if ( bus_index == 0 )
    {
        base_address_ = I2C0_BASE_ADDRESS;
    }
    else if( bus_index == 1 )
    {
        base_address_ = I2C1_BASE_ADDRESS;
    }
    else
    {
        log_error( "Invalid I2C bus index: %d", bus_index );
        return;
    }

    // Initialize I2C
    i2cps_config_ptr_ = XIicPs_LookupConfig( base_address_ );
    if ( i2cps_config_ptr_ == NULL )
    {
        log_error("I2C %d: lookup config failed\n", bus_index_ );
        //std::cout << "I2C " << bus_index << " lookup config failed\n";
        return;
    }

    int status = XIicPs_CfgInitialize( &i2c_ps_, i2cps_config_ptr_, i2cps_config_ptr_->BaseAddress );
    if ( status != XST_SUCCESS )
    {
        //std::cout << "I2C " << bus_index << " config initialization failed\n";
        log_error("I2C %d: config initialization failed\n", bus_index_ );
        return;
    }
    
    // Self Test
    status = XIicPs_SelfTest( &i2c_ps_ );
    if ( status != XST_SUCCESS )
    {
        //std::cout << "I2C " << bus_index << " self-test failed\n";
        log_error("I2C %d: self-test failed failed\n", bus_index_);
        return;
    }

    // Set clock frequency
    XIicPs_SetSClk( &i2c_ps_, I2C_CLOCK_FREQUENCY );
}

//=========================================
// Write to I2C bus.
//=========================================
int PSI2C::write( char* buffer, uint16_t length, uint16_t slave_address )
{
    int status = XST_SUCCESS;

    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        status = XIicPs_MasterSendPolled( &i2c_ps_, (u8*)buffer, length, slave_address );
        xSemaphoreGive( mutex_ );

        if (status != XST_SUCCESS)
        {
            //std::cout << "I2C " << bus_index_ << " failed to send\n";
            printf("I2C %d: failed to send\n", bus_index_);
        }
    }

    return status;
}

//=========================================
// Read from I2C bus.
//=========================================
int PSI2C::read( char* buffer, uint16_t length, uint16_t slave_address ) 
{
    int status = XST_SUCCESS;

    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        status = XIicPs_MasterRecvPolled( &i2c_ps_, (u8*)buffer, length, slave_address );
        xSemaphoreGive( mutex_ );
        
        if (status != XST_SUCCESS)
        {
            //std::cout << "I2C " << bus_index_ << " failed to receive\n";
            printf("I2C %d: failed to receive\n", bus_index_);
        }
    }
    
    return status;
}


void PSI2C::task()
{
    PSI2CReq  req;
    PSI2CResp resp;

    //char rd_data[4];
    //char wr_data[4];

    while(1)
    {
        xQueueReceive( req_queue_
                     , &req,
					 , portMAX_DELAY );
        
        if ( req.op && = 0x8000 )
        {
            read( resp.data, req.length, req.addr );
            resp.op = req.op;
            xQueueSend( resp_queue_
                      , resp,
                      , 0UL
                      )
        }
        else
        {
            write( req.data, req.length, req.addr );
        }
    }
}

static void PSI2C::create_psi2c_task()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { task(); });
    xTaskCreate( task_wrapper, name_.c_str(), 1000, &task_func, 1, NULL );
}