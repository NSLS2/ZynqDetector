/**
 * @file GermaniumZynq.hpp
 * @brief Class definition of `GermaniumZynq` — Linux version.
 * @details
 * Board-level hardware description for the Germanium detector.
 * Owns PsI2c buses. AsyncWorkers are created by GermaniumDetector.
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
#include "Zynq.hpp"
#include "PsI2c.hpp"

//===========================================================================//

class GermaniumZynq : public Zynq<GermaniumZynq>
{
public:
    GermaniumZynq( const Logger& logger );

    /**
     * @brief CRTP hook — no-op. AsyncWorkers created by GermaniumDetector.
     */
    void create_device_access_tasks_special();

    /**
     * @brief Access I2C buses for direct hardware calls.
     */
    PsI2c& i2c0() { return i2c0_; }
    PsI2c& i2c1() { return i2c1_; }

    ///< Board addresses
    static constexpr uintptr_t REG_BASE_ADDR = 0x43C00000;
    static constexpr size_t    REG_MAP_SIZE  = 0x10000;

private:
    PsI2c          i2c0_;
    PsI2c          i2c1_;
    const Logger&  logger_;
};

//===========================================================================//
