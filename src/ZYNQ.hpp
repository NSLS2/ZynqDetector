#pragma once

#include <atomic>
#include <map>
#include <string>

#include "FreeRTOS.h"
#include "semphr.h"

#include "PLInterface.hpp"

#define __FREERTOS__
//#define __LINUX__

#define REG_BASE_ADDR  0x43C00000
#define XADC_ADDR 0xF8007100

#define REG_VER        0x0





//=========================================
// ZYNQ class
//=========================================
class ZYNQ
{
private:
    Register reg_;
    std::map<std::string, PLI2CInterface> pl_i2c_interfaces_;
    std::map<std::string, PLSPIInterface> pl_spi_interfaces_;

public:
    ZYNQ( uintptr_t base_addr );

    void add_pl_i2c_interface( const std::string& name, uint32_t instr_reg, uint32_t data_reg );
    void add_pl_spi_interface( const std::string& name, uint32_t instr_reg, uint32_t data_reg );

    PLI2CInterface* get_pl_i2c_interface( const std::string& name );
    PLSPIInterface* get_pl_spi_interface( const std::string& name );
};
//=========================================
