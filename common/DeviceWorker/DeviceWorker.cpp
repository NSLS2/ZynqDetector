/**
 * @file DeviceWorker.cpp
 * @brief Member function definitions of `DeviceWorker`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "DeviceWorker.hpp"

//===========================================================================//

DeviceWorker::DeviceWorker(std::string name)
    : name_(std::move(name))
{}

//===========================================================================//

DeviceWorker::~DeviceWorker()
{
    stop();
}

//===========================================================================//

void DeviceWorker::set_handler(Handler h)
{
    handler_ = std::move(h);
}

//===========================================================================//

void DeviceWorker::start()
{
    running_ = true;
    thread_ = std::thread(&DeviceWorker::run, this);
}

//===========================================================================//

void DeviceWorker::stop()
{
    if (running_) {
        running_ = false;
        queue_.stop();
        if (thread_.joinable()) {
            thread_.join();
        }
    }
}

//===========================================================================//

std::future<uint32_t> DeviceWorker::submit(uint32_t addr, uint32_t value)
{
    Request req;
    req.addr  = addr;
    req.value = value;
    auto fut = req.result.get_future();
    queue_.push(std::move(req));
    return fut;
}

//===========================================================================//

void DeviceWorker::run()
{
    while (running_) {
        Request req = queue_.pop();
        if (!running_) break;

        uint32_t result = 0;
        if (handler_) {
            result = handler_(req.addr, req.value);
        }
        req.result.set_value(result);
    }
}

//===========================================================================//
