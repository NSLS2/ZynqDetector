#include "FreeRTOS.h"

#include "Register.hpp"
#include "PSI2C.hpp"
#include "PSXADC.hpp"
#include <GermaniumNetwork.hpp>

void GermaniumNetwork::rx_msg_proc(const UDPRxMsg &msg)
{
}

void GermaniumNetwork::tx_msg_proc(UDPTxMsg &msg)
{
    QueueSetMemberHandle_t active_queue_;

    RegisterAccessReq reg_acc_resp;
    PSI2CResp psi2c_resp;
    PSXADCResp psxadc_resp;

    UDPTxMsg msg;

    activeQueue = (QueueHandle_t)xQueueSelectFromSet(owner_.resp_queue_set_, portMAX_DELAY);

    switch (active)
    {
    case owner_.reg_access_resp_queue_:
        xQueueReceive(owner_.reg_access_resp_queue_, &reg_acc_resp, 0);
        msg.op = reg_acc_resp.op;
        msg.reg_acc_resp_data = reg_acc_resp.data;
        break;

    case owner_.psi2c_resp_queue:
        xQueueReceive(owner_.psi2c_resp_queue, &psi2c_resp, 0);
        msg.op = psi2c_resp.op;
        msg.i2c_acc_resp_data = psi2c_resp.data;
        break;

    case owner_.psxadc_resp_queue:
        xQueueReceive(owner_.psxadc_resp_queue, &psxadc_resp, 0);
        msg.op = psxadc_resp.op;
        msg.xadc_acc_resp_data = psxadc_resp.data;
        break;

    default:

        break;
    }
}

void GermaniumNetwork::udp_rx_task()
{
    uint16_t op;
    uint16_t obj;
    udp_msg_t udp_msg;

    uint8_t obj_type = 0;

    fast_access_req_t fast_access_req;
    slow_access_req_t slow_access_req;
    bulk_access_req_t bulk_access_req;

    Socket_t xUDPSocket;
    struct freertos_sockaddr xSourceAddress;
    socklen_t xSourceAddressLength = sizeof(xSourceAddress);
    uint16_t rx_msg_leng;
    uint8_t ucBuffer[MAX_UDP_MSG_LENG];

    //=============================
    // Initialize network
    //=============================
    socket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);

    while (1)
    {
        rx_msg_leng = FreeRTOS_recvfrom(xSocket, ucReceiveBuffer, sizeof(ucReceiveBuffer), 0, (struct freertos_sockaddr *)&xClientAddress, &xClientAddressLength);

        // Read UDP packet
        op = udp_msg.op >> 14;
        uint16_t obj = udp_msg.msg_id & 0x3F;
        switch (obj)
        {
        case MSG_VER:
            // send a fast_req to fast_access_task
            obj_type = FAST_ACCESS_REQ;
            break;
        default:;
        }

        switch (obj_type)
        {
        case FAST_ACCESS_REQ:
            xQueueSend(fast_access_req_queue,
                       fast_access_req,
                       0UL);
            break;

        case SLOW_ACCESS_REQ:
            xQueueSend(slow_access_req_queue,
                       slow_access_req,
                       0UL);
            break;

        case BULK_ACCESS_REQ:
            xQueueSend(bulk_access_req_queue,
                       bulk_access_req,
                       0UL);
            break;

        default:;
        }
    }
}

void udp_tx_task()
{
}

