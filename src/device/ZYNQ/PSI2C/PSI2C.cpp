//#include <iostream>
#include <stdio.h>
#include "PSI2C.hpp"

PSI2C::PSI2C( uint8_t bus_index ) : bus_index_( bus_index )
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

    char rd_data[4];
    char wr_data[4];

    auto param = static_cast<reg_access_task_param_t*>(pvParameters);

    while(1)
    {
        xQueueReceive( req_queue
                     , &req,
					 , portMAX_DELAY );
        
        if ( req.read )
        {
            read( resp.data, req.length, req.addr );
            resp.op = req.op;
            xQueueSend( req_queue_
                      , resp,
                      , 0UL
                      )
        }
        else
        {
            write( resp.data, req.length, req.addr );
        }
    }
}

static void PSI2C::task_wrapper(void* param, void (PSI2C::*task)())
{
    auto obj = statid_cast<PSI2C*>(param);
    if( obj )
    {
        obj->*task();
    }
    else
    {
        log_error("task_wrapper: Invalid cast\n");
    }
}