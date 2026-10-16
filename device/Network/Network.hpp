/**
 * @file Network.hpp
 * @brief Class template definition of `Network` — Linux version.
 * @details
 * Transport-agnostic CRTP base. Derived classes (GermaniumZMQ,
 * GermaniumUDP, etc.) implement the actual wire protocol.
 *
 * Async model: rx_thread dispatches incoming commands via a
 * non-blocking callback; tx_thread drains a response queue.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <functional>

#include "Logger.hpp"
#include "GermaniumDetectorProtocol.hpp"

//===========================================================================//

template < typename DerivedNetwork >
class Network
{
public:

    /**
     * @brief Non-blocking command dispatcher.
     * Called by rx_thread for each received message.
     * The dispatcher pushes work to per-bus AsyncWorkers.
     */
    using CommandDispatcher = std::function<void(const GermaniumProtocol::Message&)>;

    explicit Network( const Logger& logger );

    //------------------------------
    // CRTP helper
    //------------------------------
    auto& derived()             { return static_cast<DerivedNetwork&>(*this); }
    const auto& derived() const { return static_cast<const DerivedNetwork&>(*this); }

    /**
     * @brief Network initialization (transport-specific).
     */
    void network_init();

    /**
     * @brief Create network tasks / threads.
     */
    void create_network_tasks();

    /**
     * @brief Run the network rx loop (blocks).
     * @param dispatcher  Non-blocking callback for each received command.
     *
     * The derived class implements run_special() which:
     *   1. Starts the tx_thread (drains response queue)
     *   2. Enters rx loop: recv → decode → dispatcher(msg) → loop
     */
    void run(CommandDispatcher dispatcher);

    /**
     * @brief Push a reply to the response queue for the tx_thread.
     */
    void tx_reply(const GermaniumProtocol::Message& msg);

    /**
     * @brief Signal the network loop to stop.
     */
    void stop();

protected:
    const Logger& logger_;
};

//===========================================================================//

#include "Network.tpp"
