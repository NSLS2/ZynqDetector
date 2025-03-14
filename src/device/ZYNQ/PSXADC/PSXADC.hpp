#pragma once

#include <xadcps.h>

class PSXADC
{
private:
    XAdcPs         xadc_instance_ptr_;
    XAdcPs_Config *xadc_config_;
    u32            temperature_raw_;
    float          temprature_;
    u32            vcc_raw_;
    float          vcc_;

public:

    struct PSXADCReq
    {
        uint16_t op;
        uint8_t  addr;
        uint8_t  length;
        uint32_t data;
    };

    struct PSXADCResp
    {
        uint16_t op;
        uint8_t  length;
        uint32_t data;
    };

    PSXADC();
    float read_temperature();
    float read_vcc();

    // Task
    void task();
    static void task_wrapper(void* param, void (PSXADC::*task)());
};
