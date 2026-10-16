/**
 * @file Ad9252.hpp
 * @brief Class template definition of `Ad9252`.
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

template< typename DerivedNetwork >
class Ad9252
{
public:
    Ad9252( Register&            reg_
          , QueueHandle_t const  ad9252_access_req_queue
          );

    void create_device_access_tasks();

private:
    Register&             reg_;
    QueueHandle_t const   req_queue_;

    static constexpr uint32_t TASK_STACK_SIZE = 1000;
    StaticTask_t              task_tcb_;
    StackType_t               task_stack_[TASK_STACK_SIZE];
    TaskConfig                task_cfg_;

    void set_clk_skew( int chip_num, int skew );
    void ad9252_cnfg( int chip_num, int addr, int val );
    void load_reg( int chip_sel, int addr, int val );
    void send_spi_bit( int chip_sel, int data );

    void task();

};

#include "Ad9252.tpp"

//===========================================================================//

