#pragma once

#include <cstdint>
#include <memory>

#include "FreeRTOS.h"

#include "queue.h"

class Register
{
public:

    Register( uintptr_t base_addr );

    void write( uint32_t offset, uint32_t value );
    uint32_t read( uint32_t offset );

    void multi_access_start();
    void multi_access_write( uint32_t offset, uint32_t value );
    uint32_t multi_access_read( uint32_t offset );
    void multi_access_end();

    void Register::create_register_task();

private:
    uintptr_t base_addr_;
    SemaphoreHandle_t mutex_;
    ZynqDetector* owner_;
    void Register::task();
};
