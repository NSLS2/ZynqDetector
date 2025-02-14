#pragma once

#include <atomic>
#include <map>
#include <string>

#include "FreeRTOS.h"
#include "semphr.h"

#define __FREERTOS__
//#define __LINUX__

#define REG_BASE_ADDR  0x43C00000

#define REG_VER        0x0


//=========================================
// Register class
//=========================================
class Register
{
public:
    Register( uintptr_t base_addr);

    void write( uint32_t offset, uint32_t value );
    uint32_t read( uint32_t offset );

private:
    uintptr_t base_addr_;
    SemaphoreHandle_t mutex_;

};
//=========================================

//=========================================
// Interface class
//=========================================
class Interface
{
protected:
    Register reg_;
    uint32_t config_reg_;
    uint32_t instr_reg_;
    uint32_t data_reg_;
    uint32_t baud_rate_;
    TaskHandle_t task_handle_;

    void write( uint32_t instr, uint32_t data );
    uint32_t read( uint32_t instr, uint32_t data );
    void wait_for_completion();
    
public:
    Interface( Register& reg, uint32_t config_reg, uint32_t instr_reg, uint32_t data_reg, uint32_t baud_rate );
};
//=========================================

//=========================================
// I2C Interface class
//=========================================
class I2CInterface : public Interface
{
public:
    using Interface::Interface;
};
//=========================================

//=========================================
// SPI Interface class
//=========================================
class SPIInterface : public Interface
{
public:
    using Interface::Interface;
};
//=========================================


//=========================================
// FPGA class
//=========================================
class FPGA
{
private:
    std::atomic<bool> locked_ {false};

    Register reg_;
    std::map<std::string, I2CInterface> i2c_interfaces_;
    std::map<std::string, SPIInterface> spi_interfaces_;

    std::atomic<bool> reg_lock {false};

public:
    FPGA( uintptr_t base_addr );

    void add_i2c_interface( const std::string& name, uint32_t instr_reg, uint32_t data_reg );
    void add_spi_interface( const std::string& name, uint32_t instr_reg, uint32_t data_reg );

    I2CInterface* get_i2c_interface( const std::string& name );
    SPIInterface* get_spi_interface( const std::string& name );
};
//=========================================
