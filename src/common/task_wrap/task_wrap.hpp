/**
 * @file task_wrap.hpp
 * @brief Defines data type and task wrapper function used to create a task.
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
#include "task.h"

//===========================================================================//

struct TaskConfig {
    void (*entry)(void*);
    void* context;
};

//===========================================================================//

inline void task_wrapper(void* param)
{
    auto* cfg = static_cast<TaskConfig*>(param);
    cfg->entry(cfg->context);
}

//===========================================================================//

