#pragma once

#include <Network.hpp>

class GermaniumNetwork : public Network
{
public:
    typedef struct
    {
        char data[1024];
    } mars_config_req;

    typedef union
    {
        uint32_t reg_acc_req_data;
        unsigned int loads[12][14];
        struct chipstr globalstr[12];
        struct chanstr channelstr[384];
        uint32_t i2c_acc_req_data;
        uint32_t xadc_acc_req_data;

    } UDPRxMsgPayload;

    typedef struct
    {
        uint16_t id;
        uint16_t op;
        UDPRxMsgPayload payload;
    } UDPRxMsg;

    typedef union
    {
        uint32_t reg_acc_resp_data;
        uint32_t i2c_acc_resp_data;
        uint32_t xadc_acc_resp_data;

    } UDPTxMsgPayload;

    typedef struct
    {
        uint16_t id;
        uint16_t op;
        UDPTxMsgPayload payload;
    } UDPTxMsg;
    
    Socket_t socket_;

    //void rx_msg_proc() override;
    void tx_msg_proc() override;
};
