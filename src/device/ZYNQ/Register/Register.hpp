#pragma once

#include <cstdint>
#include <memory>

#include "FreeRTOS.h"

class Register
{
public:

    typedef struct
    {
        uint16_t op;
        uint32_t data;
    } RegisterAccessReq;

    typedef struct
    {
        uint16_t  op;
        uint32_t  data;
    } RegisterAccessResp;

    Register( uintptr_t base_addr );

    void write( uint32_t offset, uint32_t value );
    uint32_t read( uint32_t offset );

    void Register::task();
    void Register::create_register_task();

private:
    uintptr_t base_addr_;
    SemaphoreHandle_t mutex_;
};
