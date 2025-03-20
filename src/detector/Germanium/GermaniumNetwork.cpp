#include "FreeRTOS.h"

#include "Register.hpp"
#include "PSI2C.hpp"
#include "PSXADC.hpp"
#include <GermaniumNetwork.hpp>



uint16_t GermaniumNetwork::tx_msg_proc(UDPTxMsg &msg)
{
    QueueSetMemberHandle_t active_queue_;

    RegisterAccessReq reg_acc_resp;
    PSI2CResp psi2c_resp;
    PSXADCResp psxadc_resp;

    uint16_t msg_leng;

    activeQueue = (QueueHandle_t)xQueueSelectFromSet(owner_.resp_queue_set_, portMAX_DELAY);

    switch (active)
    {
        case owner_.reg_single_access_resp_queue_:
            xQueueReceive(owner_.reg_access_resp_queue_, &reg_single_acc_resp, 0);
            msg.op = reg_acc_resp.op;
            msg.reg_acc_resp_data = reg_acc_resp.data;
            break;

        case owner_.reg_multi_access_resp_queue_:
            xQueueReceive(owner_.reg_multi_access_resp_queue_, &reg_single_acc_resp, 0);
            msg.op = reg_acc_resp.op;
            msg.reg_acc_resp_data = reg_ac_resp.data;
            
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
