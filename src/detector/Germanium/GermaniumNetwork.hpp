/**
 * @file GermaniumNetwork.hpp
 * @brief Class template definition of `GermaniumNetwork`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include "Network.hpp"
#include "PsI2c.hpp"
#include "PsXadc.hpp"

//===========================================================================//

class GermaniumNetwork : public Network<GermaniumNetwork>
{
    using UdpRxMsg = typename Network<GermaniumNetwork>::UdpReqMsg;
    using UdpTxMsg = typename Network<GermaniumNetwork>::UdpRespMsg;

    using I2cAccessHandler = std::function<void(const UdpRxMsg&)>;

public:
    GermaniumNetwork( const QueueHandle_t   register_single_access_req_queue
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
                    );

    ///< Message/operation IDs
    static constexpr uint16_t MARS_CONF_LOAD    = 0;
    static constexpr uint16_t LEDS              = 1;
    static constexpr uint16_t MARS_CONFIG       = 2;
    static constexpr uint16_t VERSIONREG        = 3;
    static constexpr uint16_t MARS_CALPULSE     = 4;
    static constexpr uint16_t MARS_PIPE_DELAY   = 5;
    static constexpr uint16_t MARS_RDOUT_ENB    = 8;
    static constexpr uint16_t EVENT_TIME_CNTR   = 9;
    static constexpr uint16_t SIM_EVT_SEL       = 10;
    static constexpr uint16_t SIM_EVENT_RATE    = 11;
    static constexpr uint16_t ADC_SPI           = 12;
    static constexpr uint16_t CALPULSE_CNT      = 16;
    static constexpr uint16_t CALPULSE_RATE     = 17;
    static constexpr uint16_t CALPULSE_WIDTH    = 18;
    static constexpr uint16_t CALPULSE_MODE     = 19;
    static constexpr uint16_t TD_CAL            = 20;
    static constexpr uint16_t EVENT_FIFO_DATA   = 24;
    static constexpr uint16_t EVENT_FIFO_CNT    = 25;
    static constexpr uint16_t EVENT_FIFO_CTRL   = 26;
    static constexpr uint16_t DMA_CONTROL       = 32;
    static constexpr uint16_t DMA_STATUS        = 33;
    static constexpr uint16_t DMA_BASEADDR      = 34;
    static constexpr uint16_t DMA_BURSTLEN      = 35;
    static constexpr uint16_t DMA_BUFLEN        = 36;
    static constexpr uint16_t DMA_CURADDR       = 37;
    static constexpr uint16_t DMA_THROTTLE      = 38;
    static constexpr uint16_t UDP_IP_ADDR       = 40;
    static constexpr uint16_t DMA_IRQ_THROTTLE  = 48;
    static constexpr uint16_t DMA_IRQ_ENABLE    = 49;
    static constexpr uint16_t DMA_IRQ_COUNT     = 50;
    static constexpr uint16_t TRIG              = 52;
    static constexpr uint16_t COUNT_TIME        = 53;
    static constexpr uint16_t FRAME_NO          = 54;
    static constexpr uint16_t COUNT_MODE        = 55;
    static constexpr uint16_t HV                = 150;  // High voltage
    static constexpr uint16_t HV_CUR            = 151;  // High voltage current
    static constexpr uint16_t TEMP1             = 160;  // Temperature 1
    static constexpr uint16_t TEMP2             = 161;  // Temperature 2
    static constexpr uint16_t TEMP3             = 162;  // Temperature 3
    static constexpr uint16_t ZTEMP             = 170;  // CPU temperature
    static constexpr uint16_t DAC_INT_REF       = 180;  // Dac7678 internal reference

    static constexpr uint16_t STUFF_MARS        = 190;
    static constexpr uint16_t ADC_CLK_SKEW      = 191;
    static constexpr uint16_t ZDDM_ARM          = 192;

    static constexpr uint16_t DETECTOR_TYPE     = 193;
    static constexpr uint16_t COUNT_TIME_LO     = 194;
    static constexpr uint16_t COUNT_TIME_HI     = 195;

    Network<GermaniumNetwork>* base_;

    std::map<uint32_t, std::function<void(const UdpRxMsg&)>> rx_instr_map_;

    ///< Data types
    
    //-----------------------------
    struct SingleWordReqMsgPayload
    {
        uint32_t data;
    };
    //-----------------------------
    struct AdcClkSkewReqMsgPayload
    {
        uint16_t chip_num;
        uint16_t skew;
    };
    //-----------------------------
    struct StuffMarsReqMsgPayload
    {
        uint32_t loads[12][14];
    };
    //-----------------------------
    struct ZddmArmReqMsgPayload
    {
        uint16_t mode;
        uint16_t val;
    };
    //-----------------------------
    union UdpReqMsgPayload
    {
        SingleWordReqMsgPayload  single_word;
        AdcClkSkewReqMsgPayload  ad9252_clk_skew;
        StuffMarsReqMsgPayload   stuff_mars;
        ZddmArmReqMsgPayload     zddm_arm;
    };

    //-----------------------------
    struct SingleWordRespMsgPayload
    {
        uint32_t data;
    };
    //-----------------------------
    struct PsI2cRespMsgPayload
    {
        uint8_t length;
        uint8_t data[4];
    };
    //-----------------------------
    struct PsXadcRespMsgPayload
    {
        uint8_t length;
        uint8_t data[4];
    };
    union UdpRespMsgPayload
    {
        SingleWordRespMsgPayload single_word;
        PsI2cRespMsgPayload      psi2c;
        PsXadcRespMsgPayload     psxadc;
    };

    /**
     * @brief Register I2C access message handlers.
     */
    void register_i2c_handlers( const std::map<uint16_t, I2cAccessHandler>& handlers );

    /**
     * @brief Tx message process.
     */
    size_t tx_msg_proc_special( UdpTxMsg &msg );

    /**
     * @brief Rx message process.
     */
    void rx_msg_proc_special( const UdpRxMsg& msg );

protected:

    void proc_register_single_access_msg( const UdpRxMsg& msg );
    void proc_psi2c_access_msg          ( const UdpRxMsg& msg );
    void proc_psxadc_access_msg         ( const UdpRxMsg& msg );
    void proc_ad9252_access_msg         ( const UdpRxMsg& msg );
    void proc_mars_access_msg           ( const UdpRxMsg& msg );
    void proc_zddm_access_msg           ( const UdpRxMsg& msg );

private:

    const QueueHandle_t& register_single_access_req_queue_;
    const QueueHandle_t& register_single_access_resp_queue_;
    const QueueHandle_t& psi2c0_access_req_queue_;
    const QueueHandle_t& psi2c1_access_req_queue_;
    const QueueHandle_t& psi2c_access_resp_queue_;
    const QueueHandle_t& psxadc_access_req_queue_;
    const QueueHandle_t& psxadc_access_resp_queue_;
    const QueueHandle_t& ad9252_access_req_queue_;
    const QueueHandle_t& mars_access_req_queue_;
    const QueueHandle_t& zddm_access_req_queue_;

    const Logger&  logger_;

    std::map<uint16_t, I2cAccessHandler> i2c_access_dispatch_map_;
};

//===========================================================================//

using UdpRxMsg = typename Network<GermaniumNetwork>::UdpReqMsg;
using UdpTxMsg = typename Network<GermaniumNetwork>::UdpRespMsg;

//===========================================================================//

