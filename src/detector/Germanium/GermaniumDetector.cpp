/**
 * @file GermaniumDetector.cpp
 * @brief Member function definitions of `GermaniumDetector`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

// C++ includes
#include <iterator>
#include <portmacro.h>
#include <memory>

// FreeRTOS includes
#include "FreeRTOS.h"
//#include "msg.hpp"
#include "task.h"
#include "queue.h"
#include "timers.h"

// Xilinx includes
#include "xil_printf.h"
#include "xparameters.h"

// Project includes
#include "ZynqDetector.hpp"
#include "GermaniumDetector.hpp"
#include "GermaniumZynq.hpp"
#include "GermaniumNetwork.hpp"

//===========================================================================//

//static TimerHandle_t xPollTimer = NULL;

/**
 * @brief GermaniumDetector constructor
 */
GermaniumDetector::GermaniumDetector()
    : ZynqDetector< GermaniumDetector
                  , GermaniumNetwork
                  , GermaniumZynq
                  >()
{
    create_queues();

    create_components();

    network_->network_init();
 
    init_i2c_access_dispatch_map();
    network_->register_i2c_handlers( i2c_access_dispatch_map_ );
}

//===========================================================================//

/**
 * @briefe Create GermaniumDetectors specific components.
 * @details Zynq and Network objects are created here since only
 * GermaniumDetector/DerivedDetector knows how to properly create them.
 */
void GermaniumDetector::create_components_special()
{
    auto z = std::make_unique<GermaniumZynq>( register_single_access_req_queue_
                                            , register_single_access_resp_queue_
                                            , psi2c_0_access_req_queue_
                                            , psi2c_1_access_req_queue_
                                            , psi2c_access_resp_queue_
                                            , psxadc_access_req_queue_
                                            , psxadc_access_resp_queue_
                                            , this->logger_
                                            );
    this->set_zynq ( std::move(z) );

    auto n = std::make_unique<GermaniumNetwork>( register_single_access_req_queue_
                                               , register_single_access_resp_queue_
                                               , psi2c_0_access_req_queue_
                                               , psi2c_1_access_req_queue_
                                               , psi2c_access_resp_queue_
                                               , psxadc_access_req_queue_
                                               , psxadc_access_resp_queue_
                                               , ad9252_access_req_queue_
                                               , mars_access_req_queue_
                                               , zddm_access_req_queue_
                                               , this->logger_
                                               );
    this->set_network ( std::move(n) );

    ltc2309_0_ = std::make_unique<Ltc2309<PsI2c>>( /*psi2c_1_
                                                 , */LTC2309_0_I2C_ADDR
                                                 , psi2c_1_access_req_queue_
                                                 , true
                                                 , this->logger_
                                                 );
               
    ltc2309_1_ = std::make_unique<Ltc2309<PsI2c>>( /*psi2c_1_
                                                 , */LTC2309_1_I2C_ADDR
                                                 , psi2c_1_access_req_queue_
                                                 , true
                                                 , this->logger_
                                                 );
               
    dac7678_ = std::make_unique<Dac7678<PsI2c>>( /*psi2c_1_
                                               , */DAC7678_I2C_ADDR
                                               , psi2c_1_access_req_queue_
                                               , this->logger_
                                               );
              
    tmp100_0_ = std::make_unique<Tmp100<PsI2c>>( /*psi2c_0_
                                               , */TMP100_0_I2C_ADDR
                                               , psi2c_0_access_req_queue_
                                               , this->logger_
                                               );
               
    tmp100_1_ = std::make_unique<Tmp100<PsI2c>>( /*psi2c_0_
                                               , */TMP100_1_I2C_ADDR
                                               , psi2c_0_access_req_queue_
                                               , this->logger_
                                               );
               
    tmp100_2_ = std::make_unique<Tmp100<PsI2c>>( /*psi2c_0_
                                                , */TMP100_2_I2C_ADDR
                                                , psi2c_0_access_req_queue_
                                                , this->logger_
                                                );
               
    ad9252_ = std::make_unique<Ad9252<GermaniumNetwork>>( *this->zynq_->base_->reg_
                                                        , ad9252_access_req_queue_
                                                        );
             
    mars_ = std::make_unique<Mars<GermaniumNetwork>>( *this->zynq_->base_->reg_
                                                    , mars_access_req_queue_
                                                    );
           
    zddm_ = std::make_unique<Zddm<GermaniumNetwork>>( *this->zynq_->base_->reg_
                                                    , zddm_access_req_queue_
                                                    );
}

//===========================================================================//

/**
 * @brief Initialize the dispatch maps for processing I2C components.
 */
void GermaniumDetector::init_i2c_access_dispatch_map()
{
    for (const auto& [op, pair] : dac7678_instr_map_)
    {
        auto [dev, chan] = pair;
    
        i2c_access_dispatch_map_[op] = [dev, chan](const UdpRxMsg& msg)
        {
            dev->write( msg.payload.single_word.data, chan );
        };
    }
    
    for (const auto& [op, pair] : ltc2309_instr_map_)
    {
        auto [dev, chan] = pair;
    
        i2c_access_dispatch_map_[op] = [dev, chan](const UdpRxMsg& msg)
                                       {
                                           dev->read( msg.op, chan );
                                       };
    }
    
    for (const auto& [op, dev] : tmp100_instr_map_)
    {
        i2c_access_dispatch_map_[op] = [dev](const UdpRxMsg& msg)
                                       {
                                           dev->read( msg.op );
                                       };
    }
}

//===========================================================================//

/**
 * @brief Create tasks for device access.
 */
void GermaniumDetector::create_device_access_tasks_special()
{
    ad9252_->create_device_access_tasks();
    mars_->create_device_access_tasks();
    zddm_->create_device_access_tasks();

}

//===========================================================================//

/**
 * @brief Create queues for GermaniumDetector.
 */
void GermaniumDetector::create_queues_special()
{
  psi2c_0_access_req_queue_ = xQueueCreate( 5, sizeof(PsI2cAccessReq) );
  psi2c_1_access_req_queue_ = xQueueCreate( 5, sizeof(PsI2cAccessReq) );
  psxadc_access_req_queue_  = xQueueCreate( 5, sizeof(PsXadcAccessReq) );
  ad9252_access_req_queue_  = xQueueCreate( 3, sizeof(Ad9252AccessReq) );
  mars_access_req_queue_    = xQueueCreate( 3, sizeof(Ad9252AccessReq) );
  zddm_access_req_queue_    = xQueueCreate( 3, sizeof(Ad9252AccessReq) );

  psi2c_access_resp_queue_ = xQueueCreate( 10, sizeof(PsI2cAccessResp) );
  psxadc_access_resp_queue_  = xQueueCreate( 5, sizeof(PsXadcAccessResp) );
}

//===========================================================================//

/**
 * @brief 1Hz polling function.
 */
//void GermaniumDetector::polling_1s()
//{
//    UdpRxMsg msg;
//    
//    for ( auto iter : poll_list )
//    {
//        int instr = *iter;
//        auto it = instr_map_.find(instr);
//        if (it != instr_map_.end())
//        {
//            msg.op = instr;
//            it->second(msg);  // Call the corresponding function
//        }
//        else
//        {
//            std::cout << "Unknown instruction: " << instr << '\n';
//        }
//    }
//}

//===========================================================================//
