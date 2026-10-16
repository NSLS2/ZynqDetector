/**
 * @file GermaniumRegister.hpp
 * @brief FPGA register map and simulation default values.
 * @details
 * Shared between ZynqDetector (Zynq server) and ADGermaniumZMQ (IOC).
 * The server initializes simulated registers with these defaults;
 * the IOC reads them back to verify connectivity.
 *
 * Register offsets are word offsets into the FPGA AXI register space
 * at base address 0x43C00000.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <cstdint>
#include <utility>

#include "GermaniumDetectorProtocol.hpp"

//===========================================================================//
//  Register word offsets
//===========================================================================//

namespace GermaniumReg {

using GermaniumProtocol::Register::MARS_CONF_LOAD;
using GermaniumProtocol::Register::LEDS;
using GermaniumProtocol::Register::MARS_CONFIG;
using GermaniumProtocol::Register::VERSIONREG;
using GermaniumProtocol::Register::MARS_CALPULSE;
using GermaniumProtocol::Register::MARS_PIPE_DELAY;
using GermaniumProtocol::Register::DETECTOR_MODEL;
using GermaniumProtocol::Register::MARS_RDOUT_ENB;
using GermaniumProtocol::Register::EVENT_TIME_CNTR;
using GermaniumProtocol::Register::SIM_EVT_SEL;
using GermaniumProtocol::Register::SIM_EVENT_RATE;
using GermaniumProtocol::Register::ADC_SPI;
using GermaniumProtocol::Register::CALPULSE_CNT;
using GermaniumProtocol::Register::CALPULSE_RATE;
using GermaniumProtocol::Register::CALPULSE_WIDTH;
using GermaniumProtocol::Register::CALPULSE_MODE;
using GermaniumProtocol::Register::TD_CAL;
using GermaniumProtocol::Register::EVENT_FIFO_DATA;
using GermaniumProtocol::Register::EVENT_FIFO_CNT;
using GermaniumProtocol::Register::EVENT_FIFO_CTRL;
using GermaniumProtocol::Register::UDP_IP_ADDR;
using GermaniumProtocol::Register::TRIG;
using GermaniumProtocol::Register::COUNT_TIME_LO;
using GermaniumProtocol::Register::COUNT_TIME_HI;
using GermaniumProtocol::Register::FRAME_NO;
using GermaniumProtocol::Register::COUNT_MODE;

//===========================================================================//
//  Simulation default values
//
//  When the server runs in simulation mode without the hardware register
//  backend, registers are initialized to these values. The IOC can read VERSIONREG to verify
//  connectivity and confirm it is talking to the correct firmware/sim.
//
//  Format: { word_offset, default_value }
//===========================================================================//

static constexpr uint32_t SIM_VERSION       = 0x20260404;   // firmware version
static constexpr uint32_t SIM_DETECTOR_MODEL = 0x0001;       // 1 = Germanium

struct RegDefault {
    uint16_t offset;
    uint32_t value;
};

static constexpr RegDefault SIM_DEFAULTS[] = {
    { VERSIONREG,       SIM_VERSION       },
    { DETECTOR_MODEL,    SIM_DETECTOR_MODEL  },
    { MARS_CONFIG,      0x00000000        },
    { MARS_PIPE_DELAY,  0x00000005        },
    { MARS_RDOUT_ENB,   0x00000000        },
    { SIM_EVT_SEL,      0x00000000        },
    { SIM_EVENT_RATE,   0x00000000        },
    { CALPULSE_CNT,     0x00000000        },
    { CALPULSE_RATE,    0x00000000        },
    { CALPULSE_WIDTH,   0x00000000        },
    { CALPULSE_MODE,    0x00000000        },
    { TD_CAL,           0x00000000        },
    { EVENT_FIFO_CTRL,  0x00000000        },
    { TRIG,             0x00000000        },
    { COUNT_TIME_LO,    0x00000000        },
    { COUNT_TIME_HI,    0x00000000        },
    { FRAME_NO,         0x00000000        },
    { COUNT_MODE,       0x00000000        },
};

static constexpr size_t SIM_DEFAULTS_COUNT =
    sizeof(SIM_DEFAULTS) / sizeof(SIM_DEFAULTS[0]);

} // namespace GermaniumReg

//===========================================================================//
