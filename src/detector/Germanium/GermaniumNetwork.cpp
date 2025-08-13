/**
 * @file Zddm.cpp
 * @brief Member function definitions of `Zddm`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include <cstring>

#include "FreeRTOS.h"

#include "Logger.hpp"
#include "Register.hpp"
#include "PsI2c.hpp"
#include "PsXadc.hpp"
//#include "Network.hpp"
#include "GermaniumNetwork.hpp"

//===========================================================================//

GermaniumNetwork::GermaniumNetwork( const QueueHandle_t   register_single_access_req_queue
                                  , const QueueHandle_t   register_single_access_resp_queue
                                  , const QueueHandle_t   psi2c0_access_req_queue
                                  , const QueueHandle_t   psi2c1_access_req_queue
                                  , const QueueHandle_t   psi2c_access_resp_queue
                                  , const QueueHandle_t   psxadc_access_req_queue
                                  , const QueueHandle_t   psxadc_access_resp_queue
                                  , const QueueHandle_t   ad9252_access_req_queue
                                  , const QueueHandle_t   mars_access_req_queue
                                  , const QueueHandle_t   zddm_access_req_queue
                                  , const Logger&         logger
                                  )
                                  : Network<GermaniumNetwork>           ( logger                            )
                                  , base_                   ( static_cast<Network<GermaniumNetwork>*>(this) )
                                  , register_single_access_req_queue_   ( register_single_access_req_queue  )
                                  , register_single_access_resp_queue_  ( register_single_access_resp_queue )
                                  , psi2c0_access_req_queue_            ( psi2c0_access_req_queue           )
                                  , psi2c1_access_req_queue_            ( psi2c1_access_req_queue           )
                                  , psi2c_access_resp_queue_            ( psi2c_access_resp_queue           )
                                  , psxadc_access_req_queue_            ( psxadc_access_req_queue           )
                                  , psxadc_access_resp_queue_           ( psxadc_access_resp_queue          )
                                  , ad9252_access_req_queue_            ( ad9252_access_req_queue           )
                                  , mars_access_req_queue_              ( mars_access_req_queue             )
                                  , zddm_access_req_queue_              ( zddm_access_req_queue             )
                                  , logger_                             ( logger                            )
{}


//===========================================================================//

/**
 * @brief Rx message process.
 * @param msg Received UDP message.
 */
void GermaniumNetwork::rx_msg_proc_special( const UdpRxMsg& msg )
{
    switch( msg.op & 0x8000 )
    {
    // Register single access
        case (EVENT_FIFO_CTRL) :
            proc_register_single_access_msg( msg );
            break;
        case (DETECTOR_TYPE)   :
            proc_register_single_access_msg( msg );
            break;
        case (MARS_RDOUT_ENB)  :
            proc_register_single_access_msg( msg );
            break;
        case (TRIG)            :
            proc_register_single_access_msg( msg );
            break;
        case (FRAME_NO)        :
            proc_register_single_access_msg( msg );
            break;
        case (COUNT_TIME_LO)   :
            proc_register_single_access_msg( msg );
            break;
        case (COUNT_TIME_HI)   :
            proc_register_single_access_msg( msg );
            break;
        case (MARS_CONF_LOAD)  :
            proc_register_single_access_msg( msg );
            break;
        case (ADC_SPI)         :
            proc_register_single_access_msg( msg );
            break;
        case (VERSIONREG)      :
            proc_register_single_access_msg( msg );
            break;
        case (MARS_PIPE_DELAY) :
            proc_register_single_access_msg( msg );
            break;
        case (TD_CAL)          :
            proc_register_single_access_msg( msg );
            break;
        case (COUNT_MODE)      :
            proc_register_single_access_msg( msg );
            break;
        case (CALPULSE_RATE)   :
            proc_register_single_access_msg( msg );
            break;
        case (CALPULSE_WIDTH)  :
            proc_register_single_access_msg( msg );
            break;
        case (CALPULSE_CNT)    :
            proc_register_single_access_msg( msg );
            break;
        case (MARS_CALPULSE)   :
            proc_register_single_access_msg( msg );
            break;
        case (CALPULSE_MODE)   :
            proc_register_single_access_msg( msg );
            break;
        case (UDP_IP_ADDR)     :
            proc_register_single_access_msg( msg );
            break;
        case (EVENT_FIFO_CNT)  :
            proc_register_single_access_msg( msg );
            break;
        case (EVENT_FIFO_DATA) :
            proc_register_single_access_msg( msg );
            break;

        case (STUFF_MARS)      :
            proc_mars_access_msg( msg );
            break;
        case (ADC_CLK_SKEW)    :
            proc_ad9252_access_msg( msg );
            break;
        case (ZDDM_ARM)        :
            proc_zddm_access_msg( msg );
            break;

        case (HV)              :
            proc_psi2c_access_msg( msg );
            break;
        case (HV_CUR)          :
            proc_psi2c_access_msg( msg );
            break;
        case (TEMP1)           :
            proc_psi2c_access_msg( msg );
            break;
        case (TEMP2)           :
            proc_psi2c_access_msg( msg );
            break;
        case (TEMP3)           :
            proc_psi2c_access_msg( msg );
            break;
        case (DAC_INT_REF)     :
            proc_psi2c_access_msg( msg );
            break;

        case (ZTEMP)           :
            proc_psxadc_access_msg( msg );
            break;
    }
}

//===========================================================================//

/**
 * @brief Register I2C access message handlers.
 * @param handlers I2C access message handlers, created by GermaniumDetector.
 */
void GermaniumNetwork::register_i2c_handlers( const std::map<uint16_t
                                            , I2cAccessHandler>& handlers
                                            )
{
    for ( const auto& [op, fn] : handlers )
    {
        auto [it, inserted] = i2c_access_dispatch_map_.emplace(op, fn);
        if (!inserted) {
            logger_.log_error( "Duplicate opcode ", op, " detected" );
        }
    }
}

//===========================================================================//

/**
 * @brief Process register single access message.
 * @param msg Reference to the received message.
 */
void GermaniumNetwork::proc_register_single_access_msg( const UdpRxMsg& msg )
{
    RegisterSingleAccessReq req;
    req.op = msg.op;
    req.data = msg.payload.single_word.data;

    xQueueSend( register_single_access_req_queue_
              , &req
              , 0UL
              );
}

//===========================================================================//

/**
 * @brief Process AD9252 access message.
 * @param msg Reference to the received message.
 */
void GermaniumNetwork::proc_ad9252_access_msg( const UdpRxMsg& msg )
{
    Ad9252AccessReq req;
    req.op       = msg.op;
    req.chip_num = msg.payload.ad9252_clk_skew.chip_num;
    req.data     = msg.payload.ad9252_clk_skew.skew;

    xQueueSend( ad9252_access_req_queue_
              , &req
              , 0UL
              );
}

//===========================================================================//

/**
 * @brief Process MARS access message.
 * @param msg Reference to the received message.
 */
void GermaniumNetwork::proc_mars_access_msg( const UdpRxMsg& msg )
{
    MarsAccessReq req;
    std::memcpy( req.loads, msg.payload.stuff_mars.loads, sizeof(req.loads) );

    xQueueSend( mars_access_req_queue_
              , &req
              , 0UL
              );
}

//===========================================================================//

/**
 * @brief Process ZDDM access message.
 * @param msg Reference to the received message.
 */
void GermaniumNetwork::proc_zddm_access_msg( const UdpRxMsg& msg )
{
    ZddmAccessReq req;
    req.mode = msg.payload.zddm_arm.mode;
    req.val  = msg.payload.zddm_arm.val;

    xQueueSend( zddm_access_req_queue_
              , &req
              , 0UL
              );
}

//===========================================================================//

/**
 * @brief Process I2C bus access message.
 * @param msg Reference to the received message.
 */
void GermaniumNetwork::proc_psi2c_access_msg( const UdpRxMsg& msg )
{
    auto it = i2c_access_dispatch_map_.find( msg.op );

    if ( it != i2c_access_dispatch_map_.end() )
    {
        it->second( msg );
    }
    else
    {
        logger_.log_error( "Unhandled opcode: ", msg.op );
    }
}

//===========================================================================//

/**
 * @brief Process XADC access message.
 * @param msg Reference to the received message.
 */
void GermaniumNetwork::proc_psxadc_access_msg( const UdpRxMsg& msg )
{
    PsXadcAccessReq req;

    req.op = msg.op;

    switch( req.op )
    {
        case ZTEMP:
            break;

        default:
            ;
    }

    xQueueSend( psxadc_access_req_queue_
              , &req
              , 0UL
              );
}

//===========================================================================//

/**
 * @brief Process message to be sent.
 * @param msg Reference to the message to be sent.
 */
size_t GermaniumNetwork::tx_msg_proc_special( UdpTxMsg& msg )
{
    QueueSetHandle_t resp_queue_set;
    resp_queue_set = xQueueCreateSet(50);

    xQueueAddToSet( register_single_access_resp_queue_, resp_queue_set );
    xQueueAddToSet( psi2c_access_resp_queue_, resp_queue_set );
    xQueueAddToSet( psxadc_access_resp_queue_, resp_queue_set );

    QueueSetMemberHandle_t active_queue;

    active_queue = (QueueHandle_t)xQueueSelectFromSet(resp_queue_set, portMAX_DELAY);

    if ( active_queue == register_single_access_resp_queue_ )
    {
        RegisterSingleAccessResp register_single_access_resp;

        xQueueReceive( register_single_access_resp_queue_, &register_single_access_resp, 0);
        msg.op = register_single_access_resp.op;
        msg.payload.single_word.data = register_single_access_resp.data;
        
        return ( 4 + sizeof(msg.payload.single_word) );
    }

    if ( active_queue == psi2c_access_resp_queue_ )
    {
        PsI2cAccessResp psi2c_access_resp;

        xQueueReceive( psi2c_access_resp_queue_, &psi2c_access_resp, 0 );
        msg.op = psi2c_access_resp.op;
        msg.payload.psi2c.length = psi2c_access_resp.length;
        memcpy( msg.payload.psi2c.data, psi2c_access_resp.data, sizeof(psi2c_access_resp.data) );
        
        return ( 4 + sizeof(msg.payload.psi2c) );
    }

    if ( active_queue == psxadc_access_resp_queue_ )
    {
        PsXadcAccessResp psxadc_access_resp;

        xQueueReceive( psxadc_access_resp_queue_, &psxadc_access_resp, 0 );
        msg.op = psxadc_access_resp.op;
        msg.payload.psxadc.length = psxadc_access_resp.length;
        memcpy( msg.payload.psxadc.data, psxadc_access_resp.data, sizeof(psxadc_access_resp.data) );

        return ( 4 + sizeof(msg.payload.psxadc) );
    }
    return 0;
}

//===========================================================================//

