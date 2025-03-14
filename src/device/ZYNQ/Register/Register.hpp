#pragma once



class Register
{
public:

    struct RegisterReq
    {
        uint16_t op;
        uint8_t  length;
        uint8_t  addr;
        uint8_t  data[4];
    };
    
    struct RegisterResp
    {
        uint16_t op;
        uint8_t  length;
        uint8_t  data[4];
    };

    Register( uintptr_t base_addr );

    void write( uint32_t offset, uint32_t value );
    uint32_t read( uint32_t offset );

private:
    uintptr_t base_addr_;
    SemaphoreHandle_t mutex_;
};
