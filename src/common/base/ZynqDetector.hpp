/**
 * @file ZynqDetector.hpp
 * @brief Class declration of `ZynqDetector`.
 * @details
 * This file defines `ZynqDetector` class template.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//#include <vector>
//#include <cstdint>
//#include <memory>

#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "timers.h"

//#include "xil_printf.h"
//#include "xparameters.h"

#include "Logger.hpp"
//#include "Register.hpp"
#include "Network.hpp"
#include "Zynq.hpp"


template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
class ZynqDetector
{
protected:

    //==================================================
    //                    Variables                   //
    //==================================================

    //------------------------------
    // Queues
    //------------------------------
    QueueHandle_t  register_single_access_req_queue_  = NULL;
    QueueHandle_t  register_single_access_resp_queue_ = NULL;
        
    //------------------------------
    // Network
    //------------------------------
    std::unique_ptr<DerivedNetwork> network_;

    //------------------------------
    // Zynq
    //------------------------------
    std::unique_ptr<DerivedZynq>    zynq_;

    //==================================================
    //                    Functions                   //
    //==================================================

    /*
     * @brief Create network Rx/Tx tasks.
     */
    void create_network_tasks();

    /*
     * @brief Create device access tasks, mostly for
     * devices to be accessed through Zynq.
     */
    void create_device_access_tasks();

    /*
     * @brief Create polling tasks that reports detector
     * status periodically.
     */
    //void create_polling_tasks();

public:
    Logger logger_;

    ZynqDetector()  = default;
    ~ZynqDetector() = default;

    /*
     * @brief Receive zynq_ from derived detector where
     * it is created.
     */
    void set_zynq( std::unique_ptr<DerivedZynq> z );

    /*
     * @brief Receive network_ from derived detector where
     * it is created.
     */
    void set_network( std::unique_ptr<DerivedNetwork> n );
    
    /*
     * @brief Create all components.
     */
    void create_components();

    /*
     * @brief Initialize network from a configuration file.
     */
    void network_init();

    /*
     * @brief Initialize queues for request/response delivery.
     */
    void create_queues();

    /*
     * @brief Initialize all tasks.
     */
    void create_tasks();
};

#include "ZynqDetector.tpp"
