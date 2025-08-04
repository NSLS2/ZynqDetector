#pragma once

#include <cstdint>
#include <type_traits>  // for std::is_same

class PsI2c;
//class PlI2c;

//====================================
// Requests
//====================================
struct AccessReq
{
    uint16_t op;
};
const uint16_t WRITE = 0;
const uint16_t READ  = 1;

struct RegisterSingleAccessReqStruct : AccessReq
{
    uint32_t data;
};
using RegisterSingleAccessReq = RegisterSingleAccessReqStruct;


struct RegisterMultiAccessReqStruct : AccessReq
{
    uint32_t data;
};
using RegisterMultiAccessReq = RegisterMultiAccessReqStruct;


struct PsI2cAccessReqStruct : AccessReq
{
    uint8_t  length;
    uint8_t  addr;
    uint8_t  read;
    uint8_t  data[4];
};
using PsI2cAccessReq = PsI2cAccessReqStruct;


struct PsXadcAccessReqStruct : AccessReq{};
using PsXadcAccessReq= PsXadcAccessReqStruct;

struct PlInterfaceSingleAccessReqStruct : AccessReq
{
    uint16_t op;
    bool     read;
    uint8_t  device_addr;
    uint16_t instr_reg_addr;
    uint16_t data_reg_addr;
    uint32_t instr;
    uint32_t data;
};
using PlInterfaceSingleAccessReq = PlInterfaceSingleAccessReqStruct;


struct PlInterfaceMultiAccessReqStruct : AccessReq
{
    uint16_t op;
    bool     read;
    uint32_t leng;
    uint8_t  device_addr;
    uint16_t instr_reg_addr;
    uint16_t data_reg_addr;
    uint32_t instr;
    uint32_t data[4096/4 - 1];
};
using PlInterfaceMultiAccessReq = PlInterfaceMultiAccessReqStruct;

template<typename T>
struct AccessReqTypeSelector;

template<>
struct AccessReqTypeSelector<PsI2c>
{
    using type = PsI2cAccessReq;
};

//template<>
//struct AccessReqTypeSelector<PlI2c> {
//    using type = PlI2cAccessReq;
//};

//====================================
// Responses
//====================================
struct AccessResp
{
    uint16_t op;
};


struct RegisterSingleAccessRespStruct : AccessResp
{
    uint32_t  data;
};
using RegisterSingleAccessResp = RegisterSingleAccessRespStruct;


struct RegisterMultiAccessRespStruct : AccessResp
{
    uint32_t  data;
};
using RegisterMultiAccessResp = RegisterMultiAccessRespStruct;


struct PsI2cAccessRespStruct : AccessResp
{
    uint8_t  length;
    uint8_t  data[4];
};
using PsI2cAccessResp = PsI2cAccessRespStruct;

struct PsXadcAccessRespStruct : AccessResp
{
    uint8_t  length;
    uint8_t  data[4];
};
using PsXadcAccessResp = PsXadcAccessRespStruct;

struct PlInterfaceSingleAccessRespStruct : AccessResp
{
    uint16_t  op;
    uint32_t  data;
};
using PlInterfaceSingleAccessResp = PlInterfaceSingleAccessRespStruct;


struct PlInterfaceMultiAccessRespStruct : AccessResp
{
    uint16_t op;
    uint32_t leng;
    uint32_t data[4096/4 - 1];
};
using PlInterfaceMultiAccessResp = PlInterfaceMultiAccessRespStruct;


