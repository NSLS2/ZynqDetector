/**
 * @file Zynq.hpp
 * @brief Class definition of `Zynq`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <atomic>
#include <map>
#include <vector>
#include <string>

#include "FreeRTOS.h"
#include "semphr.h"

#include "Logger.hpp"
#include "PsI2c.hpp"
#include "Register.hpp"

//===========================================================================//


/**
 * @brief Zynq class
 */
template < typename DerivedZynq >
class Zynq
{
private:
    
    const QueueHandle_t  register_single_access_req_queue_;
    const QueueHandle_t  register_single_access_resp_queue_;

    std::vector<PsI2c> ps_i2cs_;
    const Logger&      logger_;

public:

    Zynq( uintptr_t            base_addr
        , const QueueHandle_t  register_single_access_req_queu
        , const QueueHandle_t  register_single_access_resp_queu
        , const Logger&        logger
        );

    std::unique_ptr<Register>    reg_ = nullptr;

    void set_register( std::unique_ptr<Register> z );

    void create_device_access_tasks();
};

//===========================================================================//

#include "Zynq.tpp"
