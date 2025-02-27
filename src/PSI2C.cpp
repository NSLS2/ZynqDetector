
#include <iostream>

#include "FreeRTOS.h"
#include "task.h"
#include "xiicps.h" // Include the Xilinx I2C driver header
#include "xparameters.h" // Include hardware parameters

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
    ConfigPtr_ = XIicPs_LookupConfig( base_address_ );
    if ( ConfigPtr_ == NULL )
    {
        std::cout << "I2C " << bus_index << " lookup config failed\n";
        vTaskDelete(NULL);
    }
    return;

    int status = XIicPs_CfgInitialize( &IicPs_, ConfigPtr_, ConfigPtr_->BaseAddress );
    if ( status != XST_SUCCESS )
    {
        std::cout << "I2C " << bus_index << " config initialization failed\n";
    }
    return;
    
    // Self Test
    status = XIicPs_SelfTest( &IicPs_ );
    if ( status != XST_SUCCESS )
    {
        std::cout << "I2C " << bus_index << " self-test failed\n";
    }

    // Set clock frequency
    XIicPs_SetSClk( &IicPs_, I2C_CLOCK_FREQUENCY );
}

int PSI2C::send( char* buffer, uint16_t length, uint16_t slave_address )
{
    int status = XIicPs_MasterSendPolled( &IicPs_, buffer, sizeof(length), slave_address );
    if (status != XST_SUCCESS)
    {
        std::cout << "I2C " << bus_index_ << " failed to send\n";
    }
    return status;
}

int PSI2C::receive( char* buffer, uint16_t length, uint16_t slave_address ) 
{
    int status = XIicPs_MasterRecvPolled( &IicPs_, buffer, length, slave_address);
    if (status != XST_SUCCESS)
    {
        std::cout << "I2C " << bus_index_ << " failed to receive\n";
    }
    
    return status;
}
