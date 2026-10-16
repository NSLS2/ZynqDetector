#include <functional>

#include "FreeRTOS.h"
#include "task.h"

#include <xparameters.h>
#include <xadcps.h>
#include <xil_types.h>
#include <xil_printf.h>
#include <sleep.h>

#include "queue.hpp"
#include "task_wrap.hpp"
#include "Logger.hpp"
#include "PsXadc.hpp"

PsXadc::PsXadc( const std::string    name
              , const QueueHandle_t  req_queue
              , const QueueHandle_t  resp_queue
              , const Logger&        logger
              )
              : name_       ( name       )
              , req_queue_  ( req_queue  )
              , resp_queue_ ( resp_queue )
              , logger_     ( logger     )
{
    int status;
    
    xadc_config_ = XAdcPs_LookupConfig( XPAR_XXADCPS_0_BASEADDR );

    status = XAdcPs_CfgInitialize( &xadc_instance_ptr_, xadc_config_, xadc_config_->BaseAddress );
    if( status != XST_SUCCESS )
    {
        xil_printf( "XADC failed to initialize.\n" );
        exit( XST_FAILURE );
    }
    
    status = XAdcPs_SelfTest( &xadc_instance_ptr_ );
    if( status != XST_SUCCESS )
    {
        xil_printf("XADC failed for self-test.\n");
        exit( XST_FAILURE );
    }
    
    // Set to safe mode
    XAdcPs_SetSequencerMode( &xadc_instance_ptr_, XADCPS_SEQ_MODE_SAFE) ;
}

//uint16_t PsXadc::read_temperature()
//{
//    auto temperature_raw_ = XAdcPs_GetAdcData( &xadc_instance_ptr_, XADCPS_CH_TEMP );
//    //temprature_      = XAdcPs_RawToTemperature( temperature_raw_ );
//    return temprature_raw;
//}
//
//uint16_t PsXadc::read_vcc()
//{
//    vcc_raw_ = XAdcPs_GetAdcData( &xadc_instance_ptr_, XADCPS_CH_VCCINT );
//    //vcc_     = XAdcPs_RawToVoltage( vcc_raw_ );
//    return vcc_raw;
//}

void PsXadc::task()
{
    PsXadcAccessReq  req;
    PsXadcAccessResp resp;

    uint16_t   val;

    //char rd_data[4];
    //char wr_data[4];

    while(1)
    {
        xQueueReceive( req_queue_
                     , &req
                     , portMAX_DELAY
                     );
        
        if ( req.op == PsXadc::PSXADC_READ_TEMPERATURE )
        {
            val = XAdcPs_GetAdcData( &xadc_instance_ptr_, XADCPS_CH_TEMP );
            //read( &resp.data, req.length, req.addr );
        }
        else if ( req.op == PsXadc::PSXADC_READ_VCC )
        {
            val = XAdcPs_GetAdcData( &xadc_instance_ptr_, XADCPS_CH_VCCINT );
            //read( &resp.data, req.length, req.addr );
        }
        resp.data[1] = static_cast<uint8_t>(val >> 8);
        resp.data[0] = static_cast<uint8_t>(val && 0xff);
        resp.op = req.op;

        xQueueSend( resp_queue_
                  , (const void*)(&resp)
                  , 0UL
                  );
    }
}

void PsXadc::create_psxadc_task()
{
    task_cfg_ = { .entry = [](void* ctx) { static_cast<PsXadc*>(ctx)->task(); },
                  .context = this
                };

    xTaskCreateStatic( task_wrapper
                     , "PsXadc"
                     , TASK_STACK_SIZE
                     , &task_cfg_
                     , TASK_PRIORITY
                     , task_stack_
                     , &task_tcb_
                     );
}
