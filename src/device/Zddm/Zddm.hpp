/**
 * @file Zddm.hpp
 * @brief Class template definition of `Zddm`.
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

//===========================================================================//

template<typename DerivedNetwork>
class Zddm
{
public:
    Zddm( Register&            reg_
        , QueueHandle_t const  zddm_access_req_queue );

    void create_device_access_tasks();

private:
    Register&            reg_;
    QueueHandle_t const  req_queue_;

    static constexpr UBaseType_t TASK_PRIORITY = 6;
    static constexpr uint32_t    TASK_STACK_SIZE = 1000;
    StaticTask_t                 task_tcb_;
    StackType_t                  task_stack_[TASK_STACK_SIZE];
    TaskConfig                   task_cfg_;

    void zddm_arm( int mode, int val );
    void task();
};

//===========================================================================//

#include "Zddm.tpp"
