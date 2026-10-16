/**
 * @file AsyncWorker.hpp
 * @brief Non-blocking per-bus worker for the async dispatch model.
 * @details
 * Each independent hardware bus gets its own AsyncWorker.
 * The rx_thread pushes GermaniumProtocol::Message requests; the worker thread
 * executes the handler and pushes replies via a callback.
 *
 * Replaces DeviceWorker for the runtime path.  DeviceWorker
 * (promise/future) is still available for synchronous init-time use.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <string>
#include <functional>
#include <thread>
#include <atomic>

#include "ThreadQueue.hpp"
#include "GermaniumDetectorProtocol.hpp"
#include "Logger.hpp"

//===========================================================================//

class AsyncWorker
{
public:
    /**
     * @brief Handler modifies msg.value in place for the response.
     */
    using Handler = std::function<void(GermaniumProtocol::Message& msg)>;

    /**
     * @brief Callback to deliver the completed reply to the tx path.
     */
    using TxSender = std::function<void(const GermaniumProtocol::Message&)>;

    AsyncWorker( std::string name, TxSender sender, const Logger& logger );
    ~AsyncWorker();

    AsyncWorker(const AsyncWorker&) = delete;
    AsyncWorker& operator=(const AsyncWorker&) = delete;

    /**
     * @brief Set the handler for this worker.
     */
    void set_handler( Handler h );

    /**
     * @brief Start the worker thread.
     */
    void start();

    /**
     * @brief Signal and join the worker thread.
     */
    void stop();

    /**
     * @brief Push a request (non-blocking).
     */
    void submit( const GermaniumProtocol::Message& msg );

private:
    std::string          name_;
    Handler              handler_;
    TxSender             sender_;
    const Logger&        logger_;
    ThreadQueue<GermaniumProtocol::Message> queue_;
    std::thread          thread_;
    std::atomic<bool>    running_{false};

    void run();
};

//===========================================================================//
