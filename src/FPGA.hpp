#pragma once

#include <atomic>

#define __FREERTOS__
//#define __LINUX__

#define REG_BASE_ADDR  0x43C00000

#define REG_VER        0x0

class FPGA
{
private:
    std::atomic<bool> locked_ {false};

    uint32_t  axi_base_addr;
    uint32_t *reg;
    size_t    reg_size;

    std::atomic<bool> reg_lock {false};

public:
    axi_reg(uint32_t axi_base_addr);
    ~axi_reg();

    uint32_t reg_rd(size_t offset);
	void reg_wr(size_t offset, uint32_t val);
    void io_wait();
};
