/**
 * @file GermaniumZMQ.cpp
 * @brief Member function definitions of `GermaniumZMQ`.
 *
 * Async PUSH-PULL transport:
 *   PULL :5555  — receives commands from IOC
 *   PUSH :5557  — sends replies to IOC
 *   rx path: recv → dispatch (non-blocking)
 *   tx path: pop tx_queue → send
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */


//===========================================================================//

#include <cstdio>
#include <cstring>

#include <zmq.h>

#include "GermaniumZMQ.hpp"

//===========================================================================//

GermaniumZMQ::GermaniumZMQ( const Logger& logger )
    : Network<GermaniumZMQ>( logger )
    , zmq_ctx_  ( nullptr )
    , zmq_rx_ ( nullptr )
    , zmq_tx_ ( nullptr )
    , running_  ( false   )
{}

//===========================================================================//

GermaniumZMQ::~GermaniumZMQ()
{
    stop_special();

    if (tx_thread_.joinable())
        tx_thread_.join();

    if (zmq_rx_) {
        zmq_close(zmq_rx_);
        zmq_rx_ = nullptr;
    }
    if (zmq_tx_) {
        zmq_close(zmq_tx_);
        zmq_tx_ = nullptr;
    }
    if (zmq_ctx_) {
        zmq_ctx_destroy(zmq_ctx_);
        zmq_ctx_ = nullptr;
    }
}

//===========================================================================//

void GermaniumZMQ::network_init_special()
{
    zmq_ctx_ = zmq_ctx_new();
    if (!zmq_ctx_) {
        logger_.log_error("GermaniumZMQ: failed to create ZMQ context");
        return;
    }

    ///< rx socket — receives commands
    zmq_rx_ = zmq_socket(zmq_ctx_, ZMQ_PULL);
    if (!zmq_rx_) {
        logger_.log_error("GermaniumZMQ: failed to create PULL socket");
        return;
    }

    char rx_ep[64];
    snprintf(rx_ep, sizeof(rx_ep), "tcp://*:%d", ZMQ_CMD_PORT);

    if (zmq_bind(zmq_rx_, rx_ep) != 0) {
        logger_.log_error("GermaniumZMQ: failed to bind PULL to %s: %s",
                          rx_ep, zmq_strerror(zmq_errno()));
        return;
    }

    logger_.log_debug("GermaniumZMQ: rx socket bound on %s", rx_ep);

    ///< tx socket — sends replies
    zmq_tx_ = zmq_socket(zmq_ctx_, ZMQ_PUSH);
    if (!zmq_tx_) {
        logger_.log_error("GermaniumZMQ: failed to create PUSH socket");
        return;
    }

    char tx_ep[64];
    snprintf(tx_ep, sizeof(tx_ep), "tcp://*:%d", ZMQ_REPLY_PORT);

    if (zmq_bind(zmq_tx_, tx_ep) != 0) {
        logger_.log_error("GermaniumZMQ: failed to bind PUSH to %s: %s",
                          tx_ep, zmq_strerror(zmq_errno()));
        return;
    }

    logger_.log_debug("GermaniumZMQ: tx socket bound on %s", tx_ep);
}

//===========================================================================//

void GermaniumZMQ::create_network_tasks_special()
{
    ///< Threads are created in run_special().
}

//===========================================================================//

void GermaniumZMQ::run_special(Network::CommandDispatcher dispatcher)
{
    running_ = true;

    ///< Start tx_thread
    tx_thread_ = std::thread( &GermaniumZMQ::tx_loop, this );

    logger_.log_debug("GermaniumZMQ: rx_thread running");

    ///< rx loop — blocks on PULL socket
    while (running_) {
        GermaniumProtocol::Message msg;
        int rc = zmq_recv(zmq_rx_, &msg, sizeof(msg), 0);
        if (rc < 0) {
            if (zmq_errno() == ETERM || zmq_errno() == EINTR) {
                break;
            }
            logger_.log_error("GermaniumZMQ: recv error: %s",
                              zmq_strerror(zmq_errno()));
            continue;
        }

        if (rc != sizeof(GermaniumProtocol::Message)) {
            logger_.log_warn("GermaniumZMQ: unexpected msg size %d (expected %zu)",
                             rc, sizeof(GermaniumProtocol::Message));
            continue;
        }

        logger_.log_debug("--------------------------------------------");
        logger_.log_debug("ZMQ RX: cmd=0x%02X addr=0x%04X value=0x%08X",
                msg.cmd, msg.addr, msg.value);
        logger_.log_debug("ZMQ RX (decoded): %s",
                GermaniumProtocol::formatMessage(msg).c_str());

        ///< Non-blocking dispatch to per-bus workers
        dispatcher(msg);
    }
}

//===========================================================================//

void GermaniumZMQ::tx_loop()
{
    logger_.log_debug("GermaniumZMQ: tx_thread running");

    while (running_) {
        GermaniumProtocol::Message msg = tx_queue_.pop();
        if (!running_) break;

        logger_.log_debug("ZMQ TX: cmd=0x%02X addr=0x%04X value=0x%08X",
                          msg.cmd, msg.addr, msg.value);
        logger_.log_debug("ZMQ TX (decoded): %s",
                          GermaniumProtocol::formatMessage(msg).c_str());

        logger_.log_debug("--------------------------------------------");
        logger_.log_debug("");

        zmq_send(zmq_tx_, &msg, sizeof(msg), 0);
    }
}

//===========================================================================//

void GermaniumZMQ::tx_reply_special(const GermaniumProtocol::Message& msg)
{
    tx_queue_.push(msg);
}

//===========================================================================//

void GermaniumZMQ::stop_special()
{
    running_ = false;

    ///< Unblock tx_thread (flush tx_queue)
    tx_queue_.stop();

    ///< Closing context will unblock zmq_recv with ETERM
    if (zmq_ctx_) {
        zmq_ctx_shutdown(zmq_ctx_);
    }
}

//===========================================================================//
