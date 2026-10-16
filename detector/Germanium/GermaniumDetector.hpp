/**
 * @file GermaniumDetector.hpp
 * @brief Class definition of `GermaniumDetector` — async Linux version.
 * @details
 * The main detector class. Inherits ZynqDevice with 3 CRTP params.
 * Owns Mars, all AsyncWorkers, and the dispatch logic.
 *
 * dispatch_command() is called by the rx_thread (non-blocking).
 * It routes each GermaniumProtocol::Message to the appropriate AsyncWorker.
 * Workers push replies via the Network::tx_reply() callback.
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

#include "ZynqDevice.hpp"
#include "GermaniumZMQ.hpp"
#include "GermaniumZynq.hpp"
#include "GermaniumRegister.hpp"
#include "Mars.hpp"
#include "AsyncWorker.hpp"
#include "GermaniumDetectorProtocol.hpp"

//===========================================================================//

class GermaniumDetector : public ZynqDevice< GermaniumDetector
                                           , GermaniumZMQ
                                           , GermaniumZynq
                                           >
{
public:
    explicit GermaniumDetector( int nelm );

    /**
     * @brief CRTP hook — no additional queues needed.
     */
    void create_queues_special();

    /**
     * @brief CRTP hook — create Zynq, Network, Mars, and other components.
     */
    void create_components_special();

    /**
     * @brief CRTP hook — create and start AsyncWorkers.
     */
    void create_tasks_special();

    /**
     * @brief Non-blocking command dispatcher (called by rx_thread).
     * Routes each GermaniumProtocol::Message to the appropriate AsyncWorker.
     * For inline ops (SET_GLOBAL, SET_CHANNEL), processes immediately
     * and pushes reply to the response queue.
     */
    void dispatch_command( const GermaniumProtocol::Message& msg );

private:
    int active_channels_;
    int active_chips_;

    std::unique_ptr<Mars>    mars_;

    ///< Per-bus async workers
    std::unique_ptr<AsyncWorker> reg_worker_;
    std::unique_ptr<AsyncWorker> mars_worker_;
    std::unique_ptr<AsyncWorker> spi_worker_;
    std::unique_ptr<AsyncWorker> i2c0_worker_;
    std::unique_ptr<AsyncWorker> i2c1_worker_;
    std::unique_ptr<AsyncWorker> xadc_worker_;

    ///< Initialization (runs before network loop)
    void init_hardware();
    void init_adc_clk_skew();
    void init_dac_reference();
    void init_mars_defaults();

    ///< ADC clock skew cache (indexed by chip_num 1..3)
    int adc_clk_skew_[4] = {0, 9, 8, 8};  ///< init defaults matching init_adc_clk_skew()

    ///< SPI bit-bang helpers (used by spi_worker handler)
    void ad9252_set_clk_skew( int chip_num, int skew );
    void ad9252_cnfg( int chip_num, int addr, int val );
    void ad9252_load_reg( int chip_sel, int addr, int data );
    void ad9252_send_spi_bit( int chip_sel, int val );
};

//===========================================================================//
