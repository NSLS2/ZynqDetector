#pragma once

#include <atomic>
#include <map>
#include <vector>
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
// Zynq class
//=========================================
class Zynq
{
private:
    
    Register reg_;
    GermaniumDetector& owner_;

public:
    Zynq( uintptr_t base_addr );

    //auto add_pl_i2c( const std::string& name, uint32_t instr_reg, uint32_t data_reg );
    //auto add_pl_spi( const std::string& name, uint32_t instr_reg, uint32_t data_reg );

    auto add_psi2c( uint8_t bus_index );

    //PLI2CInterface* get_pl_i2c_interface( const std::string& name );
    //PLSPIInterface* get_pl_spi_interface( const std::string& name );
};
//=========================================
