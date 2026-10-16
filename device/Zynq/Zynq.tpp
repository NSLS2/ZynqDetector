/**
 * @file Zynq.tpp
 * @brief Member function definitions of `Zynq` — Linux version.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include "Register.hpp"

//===========================================================================//

/**
 * @brief Zynq constructor.
 * @param base_addr  Retained for constructor compatibility; ignored by vipic backend.
 * @param map_size   Retained for constructor compatibility; ignored by vipic backend.
 * @param logger     Reference to the logger.
 */
template < typename DerivedZynq >
Zynq<DerivedZynq>::Zynq
    ( uintptr_t     base_addr
    , size_t        map_size
    , const Logger& logger
    )
    : reg_              ( std::make_unique<Register>( base_addr, map_size ) )
    , register_worker_  ( "REG" )
    , logger_           ( logger )
{
    printf("Zynq: initialized with register backend parameters base_addr=0x%08X, map_size=%zu\n", static_cast<unsigned>(base_addr), map_size);
}

//===========================================================================//

/**
 * @brief Create device access worker threads.
 * @details Configure register_worker_ handler but do NOT start it.
 * In the async model, AsyncWorkers handle runtime dispatch.
 * In the sync model, the derived class may call register_worker_.start().
 */
template < typename DerivedZynq >
void Zynq<DerivedZynq>::create_device_access_tasks()
{
    ///< Set up register worker handler (for optional sync use)
    register_worker_.set_handler(
        [this](uint32_t addr, uint32_t value) -> uint32_t {
            ///< Bit 15 set = read, else write (matches FreeRTOS convention)
            if (addr & 0x8000) {
                return reg_->read(addr & 0x7FFF);
            } else {
                reg_->write(static_cast<uint16_t>(addr), value);
                return 0;
            }
        }
    );

    ///< Derived class creates its own workers / tasks
    derived().create_device_access_tasks_special();
}

//===========================================================================//
