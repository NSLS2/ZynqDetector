#pragma once

#include "FreeRTOS.h"

#include "Register.hpp"
#include "task_wrap.hpp"

#include "queue.hpp"

//struct __attribute__((__packed__)) MarsArmStruct
//{
//    uint16_t  mode;
//    uint16_t  val;
//};
//using MarsArm = MarsArmStruct;
//using MarsAccessReq = MarsArmStruct;

template<typename DerivedNetwork>
class Mars
{
public:
    Mars( Register&            reg_
        , QueueHandle_t const  mars_access_req_queue );

    void create_device_access_tasks();

private:
    Register&            reg_;
    QueueHandle_t const  req_queue_;

    void stuff_mars( const uint32_t (&loads)[12][14] );
    void mars_cfg_task();
};

#include "Mars.tpp"
