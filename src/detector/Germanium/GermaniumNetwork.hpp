#pragma once

#include <Network.hpp>

class GermaniumNetwork : public Network
{
protected:
    void rx_msg_map_init() override;
    void tx_msg_proc( const UDPTxMsg &msg ) override;

    void proc_register_single_access_msg( const UDPRxMsg& msg );
    void proc_register_multi_access_msg( const UDPRxMsg& msg );
    void proc_update_loads_msg( const char* loads );

public:
    //===============================================================
    // Message/operation IDs
    //===============================================================
    const uint16_t MARS_CONF_LOAD    = 0;
    const uint16_t LEDS              = 1;
    const uint16_t MARS_CONFIG       = 2;
    const uint16_t VERSIONREG        = 3;
    const uint16_t MARS_CALPULSE     = 4;
    const uint16_t MARS_PIPE_DELAY   = 5;
    const uint16_t MARS_RDOUT_ENB    = 8;
    const uint16_t EVENT_TIME_CNTR   = 9;
    const uint16_t SIM_EVT_SEL       = 10;
    const uint16_t SIM_EVENT_RATE    = 11;
    const uint16_t ADC_SPI           = 12;
    const uint16_t CALPULSE_CNT      = 16;
    const uint16_t CALPULSE_RATE     = 17;
    const uint16_t CALPULSE_WIDTH    = 18;
    const uint16_t CALPULSE_MODE     = 19;
    const uint16_t TD_CAL            = 20;
    const uint16_t EVENT_FIFO_DATA   = 24;
    const uint16_t EVENT_FIFO_CNT    = 25;
    const uint16_t EVENT_FIFO_CNTRL  = 26;
    const uint16_t DMA_CONTROL       = 32;
    const uint16_t DMA_STATUS        = 33;
    const uint16_t DMA_BASEADDR      = 34;
    const uint16_t DMA_BURSTLEN      = 35;
    const uint16_t DMA_BUFLEN        = 36;
    const uint16_t DMA_CURADDR       = 37;
    const uint16_t DMA_THROTTLE      = 38;
    const uint16_t UDP_IP_ADDR       = 40;
    const uint16_t DMA_IRQ_THROTTLE  = 48;
    const uint16_t DMA_IRQ_ENABLE    = 49;
    const uint16_t DMA_IRQ_COUNT     = 50;
    const uint16_t TRIG              = 52;
    const uint16_t COUNT_TIME        = 53;
    const uint16_t FRAME_NO          = 54;
    const uint16_t COUNT_MODE        = 55;
    const uint16_t UPDATE_LOADS      = 100;
    const uint16_t STUFF_MARS        = 101;
    const uint16_t AD9252_CNFG       = 102;
    const uint16_t ZDDM_ARM          = 103;
    const uint16_t HV                = 150;  // High voltage
    const uint16_t HV_CUR            = 151;  // High voltage current
    const uint16_t TEMP1             = 160;  // Temperature 1
    const uint16_t TEMP2             = 161;  // Temperature 2
    const uint16_t TEMP3             = 162;  // Temperature 3
    const uint16_t ZTEP              = 170;  // CPU temperature
    const uint16_t DAC_INT_REF       = 180;  // DAC7678 internal reference

    //===============================================================
    // Data types
    //===============================================================
    typedef struct
    {
        uint16_t  chip_num;
        uint16_t  addr;
        uint32_t  data;
    } AD9252Cfg;
    //-----------------------------
    typedef union
    {
        uint32_t      reg_single_acc_req_data;
        unsigned int  loads[12][14];
        ZDDDMArm      zddm_arm_data;
        AD9252Cfg     ad9252_cfg_data;
        uint32_t      i2c_acc_req_data;
        uint32_t      xadc_acc_req_data;
    } UDPRxMsgPayload;
    //-----------------------------
    typedef struct
    {
        uint16_t  mode;
        uint16_t  val;
    } ZDDDMArm;
    //-----------------------------
    typedef struct
    {
        uint16_t  id;
        uint16_t  op;
        char      payload[ constexpr sizeof(UDPRxMsgPayload) ];
    } UDPRxMsg;
    //-----------------------------
    typedef union 
    {
        ZDDMArm    zddm_arm_data;
        AD9252Cfg  ad9252_cfg_data;
    } RegisterMultiAccessRequestData;
    //-----------------------------
    typedef struct
    {
        uint16_t  op;
        char      data[constexpr sizeof(RegisterMultiAccessRequestData)];
    } RegisterMultiAccessRequest;
    //-----------------------------
    typedef union
    {
        uint32_t  register_single_access_response_data;
        uint32_t  psi2c_access_response_data;
        uint32_t  psxadc_access_response_data;
    } UDPTxMsgPayload;
    //-----------------------------
    typedef struct
    {
        uint16_t  id;
        uint16_t  op;
        char      payload[ constexpr sizeof(UDPTxMsgPayload) ];
    } UDPTxMsg;
    //===============================================================    

};
