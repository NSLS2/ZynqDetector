#pragma once



class Register
{
public:
    Register( uintptr_t base_addr );

    void write( uint32_t offset, uint32_t value );
    uint32_t read( uint32_t offset );

private:
    uintptr_t base_addr_;
    SemaphoreHandle_t mutex_;
};
