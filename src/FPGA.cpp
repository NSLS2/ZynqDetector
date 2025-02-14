#include <atomic>
#include <chrono>
#include <thread>

#ifdef __LINUX__
#include <sys/mman.h>
#include <fcntl.h>  // For O_* constants
#include <unistd.h> // For close()
#endif

#include "FPGA.hpp"



//###################################################
// Definitions for FreeRTOS
//###################################################
#ifdef __FREERTOS__

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


//=========================================
// Interface class
//=========================================
Interface::Interface( Register& reg
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

void Interface::write( uint32_t instr, uint32_t data )
{
    reg_.write( data_reg_, data );
    reg_.write( instr_reg_, instr );
    wait_for_completion();
}

uint32_t Interface::read( uint32_t instr, uint32_t data )
{
    reg_.write( data_reg_, data );
    reg_.write( instr_reg_, instr );
    wait_for_completion();
    return reg_.read( data_reg_ );    // Read data
}

void Interface::wait_for_completion()
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait indefinitely for ISR notification
}
//=========================================


//-----------------------------------------
FPGA::FPGA( Register& reg )
    : reg_( reg )
{}


void FPGA::add_i2c_interface( const std::string& name, uint32_t instr_reg, uint32_t data_reg)
{
    i2c_interfaces_.emplace( std::piecewise_construct,
                             std::forward_as_tuple( name ),
                             std::forward_as_tuple( reg_, instr_reg, data_reg ) );
}

void FPGA::add_spi_interface( const std::string& name,
                              uint32_t instr_reg,
                              uint32_t data_reg )
{
    spi_interfaces_.emplace( std::piecewise_construct,
                             std::forward_as_tuple( name ),
                             std::forward_as_tuple( reg_, instr_reg, data_reg ) );
}

I2CInterface* FPGA::get_i2c_interface( const std::string& name )
{
    auto it = i2c_interfaces_.find( name );
    return ( it != i2c_interfaces_.end() ) ? &(it->second) : nullptr;
}

SPIInterface* FPGA::get_spi_interface(const std::string& name)
{
    auto it = spi_interfaces_.find( name );
    return ( it != spi_interfaces_.end() ) ? &(it->second) : nullptr;
}

//=========================================
#endif



#ifdef __LINUX__
//=========================================
// Definitions for Linux
//=========================================
FPGA::FPGA(uint32_t axi_base_addr)
    : axi_base_addr( axi_base_addr )
    , reg( nullptr )
    , reg_size( 0x10000 )
{
    int fd = open("/dev/mem",O_RDWR | O_SYNC);
    if (fd == -1)
    {
        throw std::runtime_error("Failed to open /dev/mem. Try root.");
    }

    try
    {
        //reg_size = getpagesize();
        reg = static_cast<uint32_t *>( mmap( nullptr, reg_size,
                                             PROT_READ | PROT_WRITE,
                                             MAP_SHARED,
                                             fd,
                                             axi_base_addr));
    }
    catch (const std::exception& e)
    {
        close(fd);
        throw std::runtime_error("Memory mapping failed: " + std::string(e.what()));
    }


    close(fd);
    
    if(reg == MAP_FAILED)
    {
        throw std::runtime_error("Memory mapping failed");
    }

    trace_reg( __func__,
               ": FPGA object created at 0x",
               std::hex, static_cast<void*>(reg), std::dec
             );
}

//-----------------------------------------

FPGA::~FPGA()
{
    if (reg != nullptr)
    {
        munmap(reg, reg_size);
    }
    trace_reg( __func__, ": FPGA object destructed." );
}

//-----------------------------------------

void FPGA::reg_wr(size_t offset, uint32_t value)
{
    if (offset % sizeof(uint32_t) != 0) {
        throw std::runtime_error("Offset must be aligned to register size");
    }

    volatile uint32_t *registerPtr = reg + offset / sizeof(uint32_t);
    while( reg_lock.exchange( true, std::memory_order_acquire) );
    *registerPtr = value;
    reg_lock.store( false, std::memory_order_release );


    trace_reg( __func__,
               ": write 0x", std::hex, value,
               " to 0x", offset,
               " (0x",
               static_cast<void*>(const_cast<uint32_t*>(registerPtr)),
               std::dec
             );

    reg_rd(offset);
}

//-----------------------------------------

uint32_t FPGA::reg_rd(size_t offset)
{
    uint32_t val;
    if (offset % sizeof(uint32_t) != 0)
    {
        throw std::runtime_error("Offset must be aligned to register size");
    }

    volatile uint32_t *registerPtr = reg + offset / sizeof(uint32_t);
    while( reg_lock.exchange( true, std::memory_order_acquire) );
    val = *registerPtr;
    reg_lock.store( false, std::memory_order_release );

    trace_reg( __func__, ": read 0x",
               std::hex, val,
               " @ 0x", offset,
               " (0x",
               static_cast<void*>(const_cast<uint32_t*>(registerPtr)),
               std::dec
             );

    //io_wait();
    return val;
}

//=========================================
#endif
