#include <atomic>
#include <chrono>
#include <thread>

#ifdef __LINUX__
#include <sys/mman.h>
#include <fcntl.h>  // For O_* constants
#include <unistd.h> // For close()
#endif

#include "fpga.hpp"

//=========================================
// Definitions for FreeRTOS
//=========================================
#ifdef __FREERTOS__
//-----------------------------------------
axi_reg::axi_reg(uint32_t axi_base_addr)
    : axi_base_addr( axi_base_addr )
    , reg( nullptr )
    , reg_size( 0x10000 )
{}

//-----------------------------------------

axi_reg::~axi_reg()
{}

//-----------------------------------------


//-----------------------------------------


//-----------------------------------------

//=========================================
#endif



#ifdef __LINUX__
//=========================================
// Definitions for Linux
//=========================================
axi_reg::axi_reg(uint32_t axi_base_addr)
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
               ": axi_reg object created at 0x",
               std::hex, static_cast<void*>(reg), std::dec
             );
}

//-----------------------------------------

axi_reg::~axi_reg()
{
    if (reg != nullptr)
    {
        munmap(reg, reg_size);
    }
    trace_reg( __func__, ": axi_reg object destructed." );
}

//-----------------------------------------

void axi_reg::reg_wr(size_t offset, uint32_t value)
{
    if (offset % sizeof(uint32_t) != 0) {
        throw std::runtime_error("Offset must be aligned to register size");
    }

    volatile uint32_t *registerPtr = reg + offset / sizeof(uint32_t);
    reg_lock.lock();
    *registerPtr = value;
    reg_lock.unlock();


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

uint32_t axi_reg::reg_rd(size_t offset)
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