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
