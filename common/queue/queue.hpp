/**
 * @file queue.hpp
 * @brief Constants and data types for requests/responses.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <cstdint>
#include <type_traits>  // for std::is_same

//===========================================================================//

class PsI2c;

//===========================================================================//

///< Requests

const uint16_t WRITE = 0;
const uint16_t READ  = 1;

struct RegisterSingleAccessReq
{
    uint16_t  op;
    uint32_t  data;
};

//-----------------------------

struct PsI2cAccessReq
{
    uint16_t  op;
    uint8_t   length;
    uint8_t   addr;
    uint8_t   read;
    uint8_t   data[4];
};

//-----------------------------

struct PsXadcAccessReq
{
    uint16_t  op;
    uint32_t  data;
};

//-----------------------------

struct Ad9252AccessReq
{
    uint16_t  op;
    uint16_t  chip_num;
    uint16_t  data;
};

//-----------------------------

struct ZddmAccessReq
{
    uint16_t  op;
    uint16_t  mode;
    uint16_t  val;
};

//-----------------------------

struct MarsAccessReq
{
    uint16_t  op;
    uint32_t  loads[12][14];
};

//-----------------------------

template<typename T>
struct AccessReqTypeSelector;

template<>
struct AccessReqTypeSelector<PsI2c>
{
    using type = PsI2cAccessReq;
};

//===========================================================================//

///< Responses

//-----------------------------

struct RegisterSingleAccessResp
{
    uint16_t  op;
    uint32_t  data;
};

//-----------------------------

struct PsI2cAccessResp
{
    uint16_t op;
    uint8_t  length;
    uint8_t  data[4];
};

//-----------------------------

struct PsXadcAccessResp
{
    uint16_t op;
    uint8_t  length;
    uint8_t  data[4];
};

//-----------------------------

