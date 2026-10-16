/**
 * @file GermaniumZynq.cpp
 * @brief Member function definitions of `GermaniumZynq` — Linux version.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "GermaniumZynq.hpp"

//===========================================================================//

GermaniumZynq::GermaniumZynq( const Logger& logger )
    : Zynq<GermaniumZynq>( REG_BASE_ADDR, REG_MAP_SIZE, logger )
    , i2c0_   ( 0, logger )
    , i2c1_   ( 1, logger )
    , logger_ ( logger )
{}

//===========================================================================//

void GermaniumZynq::create_device_access_tasks_special()
{
    ///< No-op: AsyncWorkers are created by GermaniumDetector.
}

//===========================================================================//
