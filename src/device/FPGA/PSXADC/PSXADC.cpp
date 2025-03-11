#include <iostream>

#include <xparameters.h>
#include <xadcps.h>
#include <xil_types.h>
#include <xil_printf.h>
#include <sleep.h>

#include "XADC.hpp"

PSXADC::PSXADC()
{
    int status;
    
    xadc_config_ = XAdcPs_LookupConfig( XPAR_XXADCPS_0_BASEADDR );

    status = XAdcPs_CfgInitialize( &xadc_instance_ptr_, xadc_config_, xadc_config_->BaseAddress );
    if( status != XST_SUCCESS )
    {
        std::cout << "XADC failed to initialize.\n";
        exit( XST_FAILURE );
    }
    
    status = XAdcPs_SelfTest( &xadc_instance_ptr_ );
    if( status != XST_SUCCESS )
    {
        print("XADC failed for self-test.\n");
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