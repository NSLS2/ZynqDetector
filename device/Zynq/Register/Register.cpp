/**
 * @file Register.cpp
 * @brief Member function definitions of `Register` — Linux version.
 * @details
 * Hardware build: /dev/vipic ioctl for FPGA register access.
 * Simulation build: heap-backed array with preset defaults.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include <cstdio>

#include "Register.hpp"

//===========================================================================//

#ifdef SIM_MODE

#include "GermaniumRegister.hpp"

Register::Register( uintptr_t /*base_addr*/, size_t map_size )
    : base_( nullptr )
{
    size_t nwords = map_size / sizeof(uint32_t);
    sim_mem_.resize( nwords, 0 );
    base_ = sim_mem_.data();

    for (size_t i = 0; i < GermaniumReg::SIM_DEFAULTS_COUNT; ++i) {
        base_[ GermaniumReg::SIM_DEFAULTS[i].offset ] =
            GermaniumReg::SIM_DEFAULTS[i].value;
    }

    printf("Register: simulation mode (heap-backed, %zu words)\n", nwords);
}

Register::~Register() {}

#else // !SIM_MODE

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace
{

static constexpr const char* VIPIC_DEVICE = "/dev/vipic";

struct pldrv_io_t
{
    uint32_t address;
    uint32_t data;
};

#define TRIGGER_DMA_TRANSFER _IO('p', 1)
#define DMA_STATUS           _IOR('p', 2, pldrv_io_t *)
#define SET_DMA_CONTROL      _IOW('p', 3, pldrv_io_t *)
#define DEBUG                _IOW('p', 4, pldrv_io_t *)
#define SET_BURST_LENGTH     _IOW('p', 5, pldrv_io_t *)
#define SET_BUFF_LENGTH      _IOW('p', 6, pldrv_io_t *)
#define SET_RATE             _IOW('p', 7, pldrv_io_t *)
#define READ_REG             _IOWR('p', 8, pldrv_io_t *)
#define WRITE_REG            _IOW('p', 9, pldrv_io_t *)

} // anonymous namespace

Register::Register( uintptr_t /*base_addr*/, size_t /*map_size*/ )
    : fd_( -1 )
{
    fd_ = open( VIPIC_DEVICE, O_RDWR );
    if ( fd_ < 0 )
    {
        perror("Register: open /dev/vipic failed");
        std::abort();
    }

    printf("Register: hardware mode (ioctl %s)\n", VIPIC_DEVICE);
}

Register::~Register()
{
    if ( fd_ >= 0 )
        close( fd_ );
}

#endif // SIM_MODE

//===========================================================================//

void Register::write( uint16_t word_offset, uint32_t value )
{
    std::lock_guard<std::mutex> lock(mutex_);
    multi_access_write( word_offset, value );
}

//===========================================================================//

uint32_t Register::read( uint16_t word_offset )
{
    std::lock_guard<std::mutex> lock(mutex_);
    return multi_access_read( word_offset );
}

//===========================================================================//

void Register::set_status( uint32_t status )
{
    write( 0xC, status );
}

//===========================================================================//

void Register::multi_access_start()
{
    mutex_.lock();
}

//===========================================================================//

void Register::multi_access_write( uint16_t word_offset, uint32_t value )
{
#ifdef SIM_MODE
    base_[word_offset] = value;
#else
    pldrv_io_t p { static_cast<uint32_t>( word_offset ), value };
    if ( ioctl( fd_, WRITE_REG, &p ) == -1 )
    {
        fprintf( stderr
               , "Register: WRITE_REG offset=%u value=0x%08X failed: %s\n"
               , static_cast<unsigned>( word_offset )
               , static_cast<unsigned>( value )
               , std::strerror( errno )
               );
    }
#endif
}

//===========================================================================//

uint32_t Register::multi_access_read( uint16_t word_offset )
{
#ifdef SIM_MODE
    return base_[word_offset];
#else
    pldrv_io_t p { static_cast<uint32_t>( word_offset ), 0xbeef };
    if ( ioctl( fd_, READ_REG, &p ) == -1 )
    {
        fprintf( stderr
               , "Register: READ_REG offset=%u failed: %s\n"
               , static_cast<unsigned>( word_offset )
               , std::strerror( errno )
               );
        return 0;
    }

    return p.data;
#endif
}

//===========================================================================//

void Register::multi_access_end()
{
    mutex_.unlock();
}

//===========================================================================//
