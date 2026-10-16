#pragma once

#include <cstdint>

#include "FreeRTOS.h"

#include "PlInterface.hpp"

//=========================================
// SPI Interface class
//=========================================
class PlSpi : public PlInterface
{
public:
    using PlInterface::PlInterface;
};
//=========================================
