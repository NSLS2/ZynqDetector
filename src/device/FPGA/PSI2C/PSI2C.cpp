//#include <iostream>
#include <stdio.h>
#include "PSI2C.hpp"

PSI2C::PSI2C( uint8_t bus_index ) : bus_index_( bus_index )
{
    if ( bus_index == 0 )
    {
        base_address_ = I2C0_BASE_ADDRESS;
    }
    else
    {
        base_address_ = I2C1_BASE_ADDRESS;
    }

    // Initialize I2C
    i2cps_config_ptr_ = XIicPs_LookupConfig( base_address_ );
    if ( i2cps_config_ptr_ == NULL )
    {
        printf("I2C %d: lookup config failed\n", bus_index_ );
        //std::cout << "I2C " << bus_index << " lookup config failed\n";
    }
    return;

    int status = XIicPs_CfgInitialize( &i2c_ps_, i2cps_config_ptr_, i2cps_config_ptr_->BaseAddress );
    if ( status != XST_SUCCESS )
    {
        //std::cout << "I2C " << bus_index << " config initialization failed\n";
        printf("I2C %d: config initialization failed\n", bus_index_ );
    }
    return;
    
    // Self Test
    status = XIicPs_SelfTest( &i2c_ps_ );
    if ( status != XST_SUCCESS )
    {
        //std::cout << "I2C " << bus_index << " self-test failed\n";
        printf("I2C %d: self-test failed failed\n", bus_index_);
    }

    // Set clock frequency
    XIicPs_SetSClk( &i2c_ps_, I2C_CLOCK_FREQUENCY );
}

//=========================================
// Send to I2C bus.
//=========================================
int PSI2C::send( char* buffer, uint16_t length, uint16_t slave_address )
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
// Receive from I2C bus.
//=========================================
int PSI2C::receive( char* buffer, uint16_t length, uint16_t slave_address ) 
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
