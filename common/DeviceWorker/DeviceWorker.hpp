/**
 * @file DeviceWorker.hpp
 * @brief Per-bus worker thread — replaces per-device FreeRTOS tasks.
 * @details
 * Each independent hardware bus/resource gets its own DeviceWorker.
 * The worker runs a single thread that processes requests from its
 * ThreadQueue. Callers get a std::future to wait on the result.
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
#include <future>
#include <cstdint>

#include "ThreadQueue.hpp"

//===========================================================================//

class DeviceWorker {
public:
    using Handler = std::function<uint32_t(uint32_t addr, uint32_t value)>;

    explicit DeviceWorker(std::string name);
    ~DeviceWorker();

    /**
     * @brief Set the handler function for this bus.
     */
    void set_handler(Handler h);

    /**
     * @brief Start the worker thread.
     */
    void start();

    /**
     * @brief Signal and join the worker thread.
     */
    void stop();

    /**
     * @brief Submit a request. Returns a future with the result.
     * @param addr  Device-specific address / offset.
     * @param value Data to send / write.
     */
    std::future<uint32_t> submit(uint32_t addr, uint32_t value);

private:
    struct Request {
        uint32_t                addr;
        uint32_t                value;
        std::promise<uint32_t>  result;
    };

    std::string             name_;
    Handler                 handler_;
    ThreadQueue<Request>    queue_;
    std::thread             thread_;
    std::atomic<bool>       running_{false};

    /**
     * @brief Thread entry — pops requests, calls handler, sets promise.
     */
    void run();
};

//===========================================================================//
