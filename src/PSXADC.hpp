#pragma once

#include <xadcps.h>

class PSXADC
{
private:
    XAdcPs         xadc_instance_ptr_;
    XAdcPs_Config  *xadc_config_;
    u32            temperature_raw_;
    float          temprature_;
    u32            vcc_raw_;
    float          vcc_;

public:
    PSXADC();
    float read_temperature();
    float read_vcc();
};
