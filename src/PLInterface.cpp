#include <unistd.h> // For close()

#include "PLInterface.hpp"


//=========================================
// Interface class
//=========================================
PLInterface::PLInterface( Register& reg
                        , uint32_t config_reg
                        , uint32_t instr_reg
                        , uint32_t data_reg
                        , uint32_t baud_rate
                        )
    : reg_        ( reg       )
    , instr_reg_  ( instr_reg )
    , data_reg_   ( data_reg  )
    , baud_rate_  ( baud_rate )
{}

void PLInterface::write( uint32_t instr, uint32_t data )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        reg_.write( data_reg_, data );
        reg_.write( instr_reg_, instr );
        wait_for_completion();
        xSemaphoreGive( mutex_ );
    }
}

uint32_t PLInterface::read( uint32_t instr, uint32_t data )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        reg_.write( data_reg_, data );
        reg_.write( instr_reg_, instr );
        wait_for_completion();
        uint32_t data = reg_.read( data_reg_ );    // Read data
        xSemaphoreGive( mutex_ );

        return data;    // Read data
}

void PLInterface::wait_for_completion()
{
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );  // Wait indefinitely for ISR notification
}
//=========================================