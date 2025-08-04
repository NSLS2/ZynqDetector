#pragma once

#include "FreeRTOS.h"

#include "Register.hpp"
#include "task_wrap.hpp"

#include "queue.hpp"

//================================================

//struct __attribute__((__packed__)) Ad9252CfgStruct
//{
//    uint16_t  chip_num;
//    //uint16_t  addr;
//    uint16_t  data;
//};
//using Ad9252Cfg = Ad9252CfgStruct;
//using Ad9252AccessReq = Ad9252CfgStruct;

//================================================

template< typename DerivedNetwork >
class Ad9252
{
public:
    Ad9252( Register&            reg_
          , QueueHandle_t const  ad9252_access_req_queue
          );

    void create_device_access_tasks();

private:
    Register&             reg_;
    QueueHandle_t const   req_queue_;

    void set_clk_skew( int chip_num, int skew );
    void ad9252_cnfg( int chip_num, int addr, int val );
    void load_reg( int chip_sel, int addr, int val );
    void send_spi_bit( int chip_sel, int data );

    void ad9252_cfg_task();

};

#include "Ad9252.tpp"
