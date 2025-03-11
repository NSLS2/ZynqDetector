#include <cstdint>

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
//=========================================
