/**
 * @file ZynqDevice.tpp
 * @brief Member function definitions of `ZynqDevice`.
 *
 * Replaces the FreeRTOS `ZynqDetector.tpp`.
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

//===========================================================================//

/**
 * @brief Receive zynq_ from derived device where it is created.
 */
template< typename DerivedDevice
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDevice< DerivedDevice
               , DerivedNetwork
               , DerivedZynq
               >::set_zynq(std::unique_ptr<DerivedZynq> z)
{
    zynq_ = std::move(z);
}

//===========================================================================//

/**
 * @brief Receive network_ from derived device where it is created.
 */
template< typename DerivedDevice
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDevice< DerivedDevice
               , DerivedNetwork
               , DerivedZynq
               >::set_network(std::unique_ptr<DerivedNetwork> n)
{
    network_ = std::move(n);
}

//===========================================================================//

/**
 * @brief Initialize network.
 */
template< typename DerivedDevice
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDevice< DerivedDevice
               , DerivedNetwork
               , DerivedZynq
               >::network_init()
{
    network_->network_init();
}

//===========================================================================//

/**
 * @brief Initialize queues / ThreadQueues.
 */
template< typename DerivedDevice
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDevice< DerivedDevice
               , DerivedNetwork
               , DerivedZynq
               >::create_queues()
{
    derived().create_queues_special();
}

//===========================================================================//

/**
 * @brief Create device access tasks / worker threads.
 */
template< typename DerivedDevice
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDevice< DerivedDevice
               , DerivedNetwork
               , DerivedZynq
               >::create_tasks()
{
    zynq_->create_device_access_tasks();

    derived().create_tasks_special();

    network_->create_network_tasks();
}

//===========================================================================//

/**
 * @brief Create components.
 */
template< typename DerivedDevice
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDevice< DerivedDevice
               , DerivedNetwork
               , DerivedZynq
               >::create_components()
{
    derived().create_components_special();
}

//===========================================================================//

/**
 * @brief Run the main command loop.
 * @details The Network's rx loop receives messages, decodes them into
 * GermaniumProtocol::Message, and calls derived().dispatch_command(msg). This method blocks.
 */
template< typename DerivedDevice
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDevice< DerivedDevice
               , DerivedNetwork
               , DerivedZynq
               >::run()
{
    network_->run(
        [this](const GermaniumProtocol::Message& msg) { derived().dispatch_command(msg); }
    );
}

//===========================================================================//

/**
 * @brief Signal shutdown.
 */
template< typename DerivedDevice
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDevice< DerivedDevice
               , DerivedNetwork
               , DerivedZynq
               >::stop()
{
    network_->stop();
}

//===========================================================================//
