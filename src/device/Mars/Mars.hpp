/**
 * @file Mars.hpp
 * @brief Class definition of `Mars`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include "FreeRTOS.h"

#include "Register.hpp"
#include "task_wrap.hpp"

#include "queue.hpp"

//===========================================================================//

template<typename DerivedNetwork>
class Mars
{
public:
    Mars( Register&            reg_
        , QueueHandle_t const  mars_access_req_queue );

    /**
     * @brief Create MARS access tasks.
     */
    void create_device_access_tasks();

private:
    Register&            reg_;
    QueueHandle_t const  req_queue_;

    static constexpr uint32_t TASK_STACK_SIZE = 1000;
    StaticTask_t              task_tcb_;
    StackType_t               task_stack_[TASK_STACK_SIZE];
    TaskConfig                task_cfg_;

    /**
     * @brief Stuff MARS.
     * @param loads Configuration data for mars.
     */
    void stuff_mars( const uint32_t (&loads)[12][14] );

    /**
     * @brief MARS access task function.
     */
    void task();
};

//===========================================================================//

#include "Mars.tpp"
