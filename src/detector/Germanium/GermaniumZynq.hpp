#pragma once

#include <atomic>
#include <map>
#include <vector>
#include <string>
#include <memory>

#include "FreeRTOS.h"
#include "semphr.h"

#include "Logger.hpp"

#include "PsI2c.hpp"
#include "PsXadc.hpp"
//#include "PlInterface.hpp"

#define __FREERTOS__
//#define __LINUX__

#define REG_VER        0x0



class GermaniumDetector;

//=========================================
// Zynq class
//=========================================
class GermaniumZynq : public Zynq<GermaniumZynq>
{

public:
    GermaniumZynq( const QueueHandle_t    register_single_access_req_queue
                 , const QueueHandle_t    register_single_access_resp_queue
                 , const QueueHandle_t    psi2c0_req_queue
                 //, const QueueHandle_t    psi2c0_resp_queue
                 , const QueueHandle_t    psi2c1_req_queue
                 , const QueueHandle_t    psi2c_resp_queue
                 //, const QueueHandle_t    psi2c1_resp_queue
                 , const QueueHandle_t    psxadc_req_queue
                 , const QueueHandle_t    psxadc_resp_queue
                 , const Logger&          logger
                 );

    Zynq<GermaniumZynq>* base_;

//    GermaniumZynq( uintptr_t base_addr );

    //auto add_pl_i2c( const std::string& name, uint32_t instr_reg, uint32_t data_reg );
    //auto add_pl_spi( const std::string& name, uint32_t instr_reg, uint32_t data_reg );

    //auto add_ps_i2c( uint8_t bus_index );

    //PlI2cInterface* get_pl_i2c_interface( const std::string& name );
    //PlSpiInterface* get_pl_spi_interface( const std::string& name );

    void create_device_access_tasks_special();

protected:
    //Register reg_;
    std::unique_ptr<PsI2c>     psi2c0_;
    std::unique_ptr<PsI2c>     psi2c1_;
    std::unique_ptr<PsXadc>    psxadc_;
    const Logger&              logger_;

    static constexpr uintptr_t REG_BASE_ADDR    = XPAR_IOBUS_0_BASEADDR;
    static constexpr uintptr_t PSI2C0_BASE_ADDR = XPAR_I2C0_BASEADDR;
    static constexpr uintptr_t PSI2C1_BASE_ADDR = XPAR_I2C1_BASEADDR;
    static constexpr uint32_t  PSI2C0_CLK_FREQ  = XPAR_I2C0_CLOCK_FREQ;
    static constexpr uint32_t  PSI2C1_CLK_FREQ  = XPAR_I2C1_CLOCK_FREQ;

    void device_access_tasks();

};
//=========================================
