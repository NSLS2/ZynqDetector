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
        uint32_t i2c_acc_req_data;
        uint32_t xadc_acc_req_data;

    } UDPRxMsgPayload;

    typedef struct
    {
        uint16_t id;
        uint16_t op;
        UDPRxMsgPayload payload;
    } UDPRxMsg;
    
    Socket_t socket_;

    void rx_msg_proc() override;
    void tx_msg_proc() override;
};
