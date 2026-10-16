/**
 * @file ZynqDevice.hpp
 * @brief Class declaration of `ZynqDevice`.
 * @details
 * This file defines the `ZynqDevice` class template — the CRTP base for
 * all Zynq/ZynqMP Linux devices (detectors, motor controllers, etc.).
 *
 * Replaces the FreeRTOS `ZynqDetector` template.
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
#include <atomic>

#include "Logger.hpp"
#include "Network.hpp"
#include "Zynq.hpp"
#include "GermaniumDetectorProtocol.hpp"

//===========================================================================//

template< typename DerivedDevice
        , typename DerivedNetwork
        , typename DerivedZynq
        >
class ZynqDevice
{
protected:

    //==================================================
    //                    Variables
    //==================================================

    //------------------------------
    // Network
    //------------------------------
    std::unique_ptr<DerivedNetwork> network_;

    //------------------------------
    // Zynq
    //------------------------------
    std::unique_ptr<DerivedZynq>    zynq_;

    //==================================================
    //                    CRTP helper
    //==================================================
    auto& derived()             { return static_cast<DerivedDevice&>(*this); }
    const auto& derived() const { return static_cast<const DerivedDevice&>(*this); }

public:
    Logger logger_;

    ZynqDevice()  = default;
    ~ZynqDevice() = default;

    /**
     * @brief Receive zynq_ from derived device where it is created.
     */
    void set_zynq( std::unique_ptr<DerivedZynq> z );

    /**
     * @brief Receive network_ from derived device where it is created.
     */
    void set_network( std::unique_ptr<DerivedNetwork> n );
    
    /**
     * @brief Create all components.
     */
    void create_components();

    /**
     * @brief Initialize network.
     */
    void network_init();

    /**
     * @brief Initialize queues for request/response delivery.
     */
    void create_queues();

    /**
     * @brief Initialize all tasks / worker threads.
     */
    void create_tasks();

    /**
     * @brief Run the main command loop (blocks).
     */
    void run();

    /**
     * @brief Signal shutdown.
     */
    void stop();
};

//===========================================================================//

#include "ZynqDevice.tpp"
