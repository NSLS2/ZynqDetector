#pragma once

#include <cstdint>


//====================================
// Requests
//====================================
struct AccessReq
{
    uint16_t op;
};


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


struct PSI2CAccessReqStruct : AccessReq
{
    uint8_t  length;
    uint8_t  addr;
    uint8_t  read;
    uint8_t  data[4];
};
using PSI2CAccessReq = PSI2CAccessReqStruct;


struct PLInterfaceSingleAccessReqStruct : AccessReq
{
    uint16_t op;
    bool     read;
    uint8_t  device_addr;
    uint16_t instr_reg_addr;
    uint16_t data_reg_addr;
    uint32_t instr;
    uint32_t data;
};
using PLInterfaceSingleAccessReq = PLInterfaceSingleAccessReqStruct;


struct PLInterfaceMultiAccessReqStruct : AccessReq
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
using PLInterfaceMultiAccessReq = PLInterfaceMultiAccessReqStruct;

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


struct PSI2CAccessRespStruct : AccessResp
{
    uint8_t  length;
    uint8_t  data[4];
};
using PSI2CAccessResp = PSI2CAccessRespStruct;


struct PLInterfaceSingleAccessRespStruct : AccessResp
{
    uint16_t  op;
    uint32_t  data;
};
using PLInterfaceSingleAccessResp = PLInterfaceSingleAccessRespStruct;


struct PlInterfaceMultiAccessRespStruct : AccessResp
{
    uint16_t op;
    uint32_t leng;
    uint32_t data[4096/4 - 1];
};
using PlInterfaceMultiAccessResp = PlInterfaceMultiAccessRespStruct;