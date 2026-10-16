/**
 * @file GermaniumZMQ.hpp
 * @brief Class definition of `GermaniumZMQ` — async PUSH-PULL transport.
 * @details
 * Implements Network<GermaniumZMQ> using ZMQ PUSH-PULL sockets.
 *   - PULL on port 5555: receives commands from IOC
 *   - PUSH on port 5557: sends replies to IOC
 *   - rx_thread: recv → dispatch (non-blocking)
 *   - tx_thread: pop tx_queue → send
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <atomic>
#include <functional>
#include <thread>
#include <cstdint>

#include "Network.hpp"
#include "GermaniumDetectorProtocol.hpp"
#include "ThreadQueue.hpp"

//===========================================================================//

class GermaniumZMQ : public Network<GermaniumZMQ>
{
public:
    static constexpr int ZMQ_CMD_PORT   = 5555;
    static constexpr int ZMQ_REPLY_PORT = 5557;

    explicit GermaniumZMQ( const Logger& logger );
    ~GermaniumZMQ();

    /**
     * @brief CRTP hook — initialize ZMQ context and sockets.
     */
    void network_init_special();

    /**
     * @brief CRTP hook — no-op (threads created in run_special).
     */
    void create_network_tasks_special();

    /**
     * @brief CRTP hook — start tx_thread, run rx loop (blocks).
     * @param dispatcher  Non-blocking callback for each received command.
     */
    void run_special(Network::CommandDispatcher dispatcher);

    /**
     * @brief CRTP hook — enqueue a reply on the tx queue.
     */
    void tx_reply_special(const GermaniumProtocol::Message& msg);

    /**
     * @brief CRTP hook — signal shutdown.
     */
    void stop_special();

private:
    void*                    zmq_ctx_;
    void*                    zmq_rx_;    ///< rx socket on cmd port
    void*                    zmq_tx_;    ///< tx socket on reply port
    std::atomic<bool>        running_;
    ThreadQueue<GermaniumProtocol::Message>   tx_queue_;
    std::thread              tx_thread_;

    /**
     * @brief tx_thread entry — drains tx_queue_, sends via tx socket.
     */
    void tx_loop();
};

//===========================================================================//
