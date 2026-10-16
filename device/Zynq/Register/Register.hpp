/**
 * @file Register.hpp
 * @brief Class definition of `Register` — Linux version.
 * @details
 * FPGA register access via /dev/vipic ioctl + std::mutex.
 * Replaces the FreeRTOS version (xSemaphore + task).
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <cstdint>
#include <cstddef>
#include <mutex>

#ifdef SIM_MODE
#include <vector>
#endif

//===========================================================================//

class Register
{
public:
    Register( uintptr_t base_addr, size_t map_size );
    ~Register();

    Register(const Register&) = delete;
    Register& operator=(const Register&) = delete;

    /**
     * @brief Single register write (mutex-protected).
     */
    void write( uint16_t word_offset, uint32_t value );

    /**
     * @brief Single register read (mutex-protected).
     */
    uint32_t read( uint16_t word_offset );

    /**
     * @brief Start a multi-access sequence (acquires mutex).
     */
    void multi_access_start();

    /**
     * @brief Write during a multi-access sequence (no mutex).
     */
    void multi_access_write( uint16_t word_offset, uint32_t value );

    /**
     * @brief Read during a multi-access sequence (no mutex).
     */
    uint32_t multi_access_read( uint16_t word_offset );

    /**
     * @brief End a multi-access sequence (releases mutex).
     */
    void multi_access_end();

    /**
     * @brief Write to status register.
     */
    void set_status( uint32_t status );

private:
#ifdef SIM_MODE
    uint32_t*              base_;
    std::vector<uint32_t>  sim_mem_;
#else
    int                    fd_;
#endif
    std::mutex             mutex_;
};

//===========================================================================//
