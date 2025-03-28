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

    void Register::task();
    void Register::create_register_task();

private:
    uintptr_t base_addr_;
    SemaphoreHandle_t mutex_;
    ZynqDetector* owner_;
};
