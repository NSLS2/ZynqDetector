#include <unistd.h> // For close()

#include "PLInterface.hpp"


//=========================================
// Interface class
//=========================================
PLInterface::PLInterface( Register& reg
                        , uint32_t config_reg
                        , uint32_t instr_reg
                        , uint32_t wr_data_reg
                        , uint32_t rd_data_reg
                        , uint32_t baud_rate_reg
                        , uint32_t baud_rate
                        )
    : reg_           ( reg          )
    , instr_reg_     ( instr_reg    )
    , wr_data_reg_   ( wr_data_reg  )
    , rd_data_reg_   ( rd_data_reg  )
    , baud_rate_reg_ ( baud_rate_reg )
    , baud_rate_     ( baud_rate    )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        reg_.write( baud_rate_reg, baud_rate );
        xSemaphoreGive( mutex_ );
    }

}

void PLInterface::write( uint32_t instr, uint32_t data )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        reg_.write( wr_data_reg_, data );
        reg_.write( instr_reg_, instr );
        wait_for_completion();
        xSemaphoreGive( mutex_ );
    }
}

uint32_t PLInterface::read( uint32_t instr, uint32_t data )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        reg_.write( wr_data_reg_, data );
        reg_.write( instr_reg_, instr );
        wait_for_completion();
        uint32_t data = reg_.read( rd_data_reg_ );    // Read data
        xSemaphoreGive( mutex_ );

        return data;    // Read data
    }
}

void set_baud_rate( uint32_t baud_rate )
{
    if ( xSemaphoreTake( mutex_, portMAX_DELAY ) == pdTRUE )
    {
        reg_.write( baud_rate_reg, baud_rate );
        xSemaphoreGive( mutex_ );
    }
}

void PLInterface::wait_for_completion()
{
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );  // Wait indefinitely for ISR notification
}
//=========================================