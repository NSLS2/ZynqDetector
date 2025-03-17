#include "FreeRTOS.h"
#include "task.h"

#include <xparameters.h>
#include <xadcps.h>
#include <xil_types.h>
#include <xil_printf.h>
#include <sleep.h>
#include <task_wrapper.hpp>
#include "Logger.hpp"
#include "XADC.hpp"

PSXADC::PSXADC( std::string name )
    : name_( name )
{
    int status;
    
    xadc_config_ = XAdcPs_LookupConfig( XPAR_XXADCPS_0_BASEADDR );

    status = XAdcPs_CfgInitialize( &xadc_instance_ptr_, xadc_config_, xadc_config_->BaseAddress );
    if( status != XST_SUCCESS )
    {
        log_error( "XADC failed to initialize.\n" );
        exit( XST_FAILURE );
    }
    
    status = XAdcPs_SelfTest( &xadc_instance_ptr_ );
    if( status != XST_SUCCESS )
    {
        log_error("XADC failed for self-test.\n");
        exit( XST_FAILURE );
    }
    
    // Set to safe mode
    XAdcPs_SetSequencerMode( &xadc_instance_ptr_, XADCPS_SEQ_MODE_SAFE) ;
}

float PSXADC::read_temperature()
{
    temperature_raw_ = XAdcPs_GetAdcData( &xadc_instance_ptr_, XADCPS_CH_TEMP );
    temprature_      = XAdcPs_RawToTemperature( temperature_raw_ );
    return temprature_;
}

float PSXADC::read_vcc()
{
    vcc_raw_ = XAdcPs_GetAdcData( &xadc_instance_ptr_, XADCPS_CH_VCCINT );
    vcc_     = XAdcPs_RawToVoltage( vcc_raw_ );
    return vcc_;
}

void PSXADC::task()
{
    PSXADCReq req;
    PSXADCResp resp;

    char rd_data[4];
    char wr_data[4];

    auto param = static_cast<reg_access_task_param_t*>(pvParameters);

    while(1)
    {
        xQueueReceive( req_queue
                     , &req,
					 , portMAX_DELAY );
        
        if ( req.op == READ_TEMPERATURE )
        {
            read( &resp.data, req.length, req.addr );
            resp.op = req.op;
            xQueueSend( req_queue_
                      , resp,
                      , 0UL
                      );
        }
        else if ( req.op == READ_VCC )
        {
            read( &resp.data, req.length, req.addr );
            resp.op = req.op;
            xQueueSend( req_queue_
                      , resp,
                      , 0UL
                      )
        }
    }
}

/*
static void PSXADC::task_wrapper(void* param, void (PSXADC::*task)())
{
    auto obj = statid_cast<PSXADC*>(param);
    if( obj )
    {
        obj->*task();
    }
    else
    {
        log_error("task_wrapper: Invalid cast\n");
    }
}
*/

void PSXADC::create_psxadc_task()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { task(); });
    xTaskCreate( task_wrapper, name_.c_str(), 1000, &task_func, 1, NULL );
}