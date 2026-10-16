/**
 * @file Zynq.hpp
 * @brief Class definition of `Zynq` — Linux version.
 * @details
 * CRTP base that describes Zynq hardware configuration.
 * Owns Register and register_worker_.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <memory>

#include "Logger.hpp"
#include "Register.hpp"
#include "DeviceWorker.hpp"

//===========================================================================//

template < typename DerivedZynq >
class Zynq
{
public:

    Zynq( uintptr_t     base_addr
        , size_t        map_size
        , const Logger& logger
        );

    //------------------------------
    // CRTP helper
    //------------------------------
    auto& derived()             { return static_cast<DerivedZynq&>(*this); }
    const auto& derived() const { return static_cast<const DerivedZynq&>(*this); }

    /**
     * @brief Access the register interface.
     */
    Register& reg() { return *reg_; }

    /**
     * @brief Access the register DeviceWorker.
     */
    DeviceWorker& register_worker() { return register_worker_; }

    /**
     * @brief Create device access worker threads.
     * Starts register_worker_, then calls derived special.
     */
    void create_device_access_tasks();

private:
    std::unique_ptr<Register>  reg_;
    DeviceWorker               register_worker_;
    const Logger&              logger_;
};

//===========================================================================//

#include "Zynq.tpp"
