#pragma once

#include <cstdint>

struct AccessReq
{
    uint16_t op;
};

struct AccessResp
{
    uint16_t op;
};

struct RegisterSingleAccessReq : public AccessReq
{
    uint32_t data;
};

struct RegisterSingleAccessResp : public AccessResp
{
    uint32_t  data;
};

struct RegisterMultiAccessReq : public AccessReq
{
    uint32_t data;
};

struct RegisterMultiAccessResp : public AccessResp
{
    uint32_t  data;
};


struct PSI2CAccessReq : public AccessReq
{
    uint8_t  length;
    uint8_t  addr;
    uint8_t  read;
    uint8_t  data[4];
};

struct PSI2CAccessResp : public AccessResp
{
    uint8_t  length;
    uint8_t  data[4];
};
