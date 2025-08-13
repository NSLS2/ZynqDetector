#pragma once

#include "task.h"
#include <xadcps.h>

#include "queue.hpp"

//==================================================

//struct PsXadcAccessReqStruct : AccessReq
//{
//    uint32_t data;
//};
//using PsXadcAccessReq= PsXadcAccessReqStruct;
//
//struct PsXadcAccessRespStruct : AccessResp
//{
//    uint8_t  length;
//    uint8_t  data[4];
//};
//using PsXadcAccessResp = PsXadcAccessRespStruct;

//==================================================

class PsXadc
{
private:
    std::string    name_;
    QueueHandle_t  req_queue_;
    QueueHandle_t  resp_queue_;

    static constexpr UBaseType_t TASK_PRIORITY   = 5;
    static constexpr uint32_t    TASK_STACK_SIZE = 1000;
    StaticTask_t                 task_tcb_;
    StackType_t                  task_stack_[TASK_STACK_SIZE];
    TaskConfig                   task_cfg_;

    XAdcPs         xadc_instance_ptr_;
    XAdcPs_Config* xadc_config_;

    const Logger&  logger_;
    //u32            temperature_raw_;
    //float          temprature_;
    //u32            vcc_raw_;
    //float          vcc_;

    //float read_temperature();
    //float read_vcc();

    void task();

public:

    static constexpr uint16_t PSXADC_READ_TEMPERATURE = 0;
    static constexpr uint16_t PSXADC_READ_VCC         = 1;

    //struct PsXadcReq
    //{
    //    uint16_t op;
    //    uint8_t  addr;
    //    uint8_t  length;
    //    uint32_t data;
    //};

    //struct PsXadcResp
    //{
    //    uint16_t op;
    //    uint8_t  length;
    //    uint32_t data;
    //};

    PsXadc( const std::string     name
          , const QueueHandle_t   req_queue
          , const QueueHandle_t   resp_queue
          , const Logger&         logger_
          );

    // Task
    void create_psxadc_task();
};
