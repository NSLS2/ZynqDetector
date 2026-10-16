/**
 * @file AsyncWorker.cpp
 * @brief Member function definitions of `AsyncWorker`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "AsyncWorker.hpp"

#include <cstdio>

//===========================================================================//

AsyncWorker::AsyncWorker( std::string name, TxSender sender, const Logger& logger )
    : name_   ( std::move(name)   )
    , sender_ ( std::move(sender) )
    , logger_ ( logger            )
{}

//===========================================================================//

AsyncWorker::~AsyncWorker()
{
    stop();
}

//===========================================================================//

void AsyncWorker::set_handler( Handler h )
{
    handler_ = std::move(h);
}

//===========================================================================//

void AsyncWorker::start()
{
    running_ = true;
    thread_  = std::thread( &AsyncWorker::run, this );
    logger_.log_debug( "AsyncWorker[%s]: started", name_.c_str() );
}

//===========================================================================//

void AsyncWorker::stop()
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

void AsyncWorker::submit( const GermaniumProtocol::Message& msg )
{
    queue_.push( msg );
}

//===========================================================================//

void AsyncWorker::run()
{
    while (running_) {
        GermaniumProtocol::Message msg = queue_.pop();
        if (!running_) break;

        if (handler_) {
            logger_.log_debug( "AsyncWorker[%s]: handling message", name_.c_str() );
            handler_( msg );
        }

        sender_( msg );
    }
}

//===========================================================================//
