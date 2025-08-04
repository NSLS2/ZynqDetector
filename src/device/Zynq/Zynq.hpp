#pragma once

#include <atomic>
#include <map>
#include <vector>
#include <string>

#include "FreeRTOS.h"
#include "semphr.h"

#include "Logger.hpp"
#include "PsI2c.hpp"
#include "Register.hpp"
//#include "PlInterface.hpp"

#define __FREERTOS__
//#define __LINUX__

//#define REG_BASE_ADDR  0x43C00000
//#define XADC_ADDR 0xF8007100

#define REG_VER        0x0



//=========================================
// Zynq class
//=========================================
template < typename DerivedZynq >
class Zynq
{
private:
    
    const QueueHandle_t  register_single_access_req_queue_;
    const QueueHandle_t  register_single_access_resp_queue_;

    std::vector<PsI2c> ps_i2cs_;
    const Logger&      logger_;

public:

    Zynq( uintptr_t            base_addr
        , const QueueHandle_t  register_single_access_req_queu
        , const QueueHandle_t  register_single_access_resp_queu
        , const Logger&        logger
        );

    std::unique_ptr<Register>    reg_ = nullptr;

    void set_register( std::unique_ptr<Register> z );

    //auto add_pl_i2c( const std::string& name, uint32_t instr_reg, uint32_t data_reg );
    //auto add_pl_spi( const std::string& name, uint32_t instr_reg, uint32_t data_reg );

    auto add_ps_i2c_interface(
        uint8_t       bus_index,
        std::string   name,
        uint32_t      device_id,
        uint32_t      base_addr,
        uint32_t      clk_freq,
        const QueueHandle_t req_queue,
        const QueueHandle_t resp_queue );

    //PlI2cInterface* get_pl_i2c_interface( const std::string& name );
    //PlSpiInterface* get_pl_spi_interface( const std::string& name );
    void create_device_access_tasks();
};
//=========================================

#include "Zynq.tpp"
