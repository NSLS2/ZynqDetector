/**
 * @file Zynq.tpp
 * @brief Member function definitions of `Zynq`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//
#pragma once

#include <atomic>
#include <chrono>
#include <thread>

#include "Register.hpp"

//===========================================================================//

/**
 * @brief Zynq constructor.
 * @param base_addr Base address of the register.
 * @param register_single_access_req_queue Queue for pass register single
 *                                         access requests.
 * @param register_single_access_reisp_queue Queue for pass register single
 *                                           access responses.
 * @param logger Reference to the logger.
 */
template < typename DerivedZynq >
Zynq<DerivedZynq>::Zynq
    ( uintptr_t            base_addr
    , const QueueHandle_t  register_single_access_req_queue
    , const QueueHandle_t  register_single_access_resp_queue
    , const Logger&        logger
    )
    : reg_ ( std::make_unique<Register>( base_addr
                                       , register_single_access_req_queue
                                       , register_single_access_resp_queue
                                       , logger
                                       )
           )
    , register_single_access_req_queue_  ( register_single_access_req_queue  )
    , register_single_access_resp_queue_ ( register_single_access_resp_queue )
    , logger_                            ( logger                            )
{}

//===========================================================================//

/**
 * @brief Create Zynq-associated device access tasks.
 */
template < typename DerivedZynq >
void Zynq<DerivedZynq>::create_device_access_tasks()
{
    reg_->create_register_access_tasks();

    static_cast<DerivedZynq*>(this)->create_device_access_tasks_special();
}

//===========================================================================//

