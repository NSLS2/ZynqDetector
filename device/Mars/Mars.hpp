/**
 * @file Mars.hpp
 * @brief MARS ASIC configuration management — Linux version.
 * @details
 * Maintains chipstr/chanstr configuration state, packs into the
 * hardware bitstream via wrap(), and writes to MARS ASICs via
 * stuff_mars() using Register's multi_access interface.
 *
 * Replaces the FreeRTOS template Mars<DerivedNetwork> with a
 * concrete class that takes Register& directly.
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

#include "Register.hpp"
#include "Logger.hpp"
#include "GermaniumDetectorProtocol.hpp"

//===========================================================================//

static constexpr int MAX_NCHIPS        = 12;
static constexpr int MAX_CHANNELS      = 384;   // 12 × 32
static constexpr int CHANNELS_PER_CHIP = 32;

//===========================================================================//
//  MARS ASIC configuration structures
//===========================================================================//

struct chipstr
{
    unsigned int  pa;       // Threshold DAC (10 bits)
    unsigned int  pb;       // Test pulse DAC (10 bits)
    unsigned char rm;       // Readout mode: 1=synch, 0=asynch
    unsigned char senfl1;   // Lock on peak found
    unsigned char senfl2;   // Lock on threshold
    unsigned char m0;       // 1=channel mon, 0=others
    unsigned char m1;       // 1=pk det on PD/PN
    unsigned char sbn;      // Enable buffer on pdn & mon outputs
    unsigned char sb;       // Enable buffer on pd & mon outputs
    unsigned char sl;       // 0=internal 2pA leakage, 1=disabled
    unsigned char ts;       // Shaping time (2 bits)
    unsigned char rt;       // 1=timing ramp duration x3
    unsigned char spur;     // 1=enable pileup rejector
    unsigned char sse;      // 1=enable multiple-firing suppression
    unsigned char tr;       // Timing ramp adjust (2 bits)
    unsigned char ss;       // Multiple firing time adjust (2 bits)
    unsigned char c;        // Monitor select (5 bits)
    unsigned char g;        // Gain select (2 bits)
    unsigned char slh;      // Internal leakage adjust
    unsigned char sp;       // Input polarity: 1=positive, 0=negative
    unsigned char saux;     // Enable monitor output
    unsigned char sbm;      // Enable output monitor buffer
    unsigned char tm;       // Timing mode: 0=ToA, 1=ToT
};

struct chanstr
{
    unsigned char dp;       // Pileup rejector trim DAC (4 bits)
    unsigned char nc1;      // No connection (set 0)
    unsigned char da;       // Threshold trim DAC (3 bits)
    unsigned char sel;      // 1=leakage current, 0=shaper output
    unsigned char nc2;      // No connection (set 0)
    unsigned char sm;       // 1=channel DISABLE
    unsigned char st;       // 1=enable test input (30fF cap)
};

//===========================================================================//
//  MARS field IDs (must match ADGermaniumZMQ enums)
//===========================================================================//

using GermaniumProtocol::MarsGlobalField;
using GermaniumProtocol::MarsChannelField;
using GermaniumProtocol::MARS_FIELD_ST;
using GermaniumProtocol::MARS_FIELD_GAIN;
using GermaniumProtocol::MARS_FIELD_POL;
using GermaniumProtocol::MARS_FIELD_EBLK;
using GermaniumProtocol::MARS_FIELD_GMON;
using GermaniumProtocol::MARS_FIELD_PUEN;
using GermaniumProtocol::MARS_FIELD_MFS;
using GermaniumProtocol::MARS_FIELD_TDS;
using GermaniumProtocol::MARS_FIELD_TDM;
using GermaniumProtocol::MARS_FIELD_TH;
using GermaniumProtocol::MARS_FIELD_TPAMP;
using GermaniumProtocol::MARS_FIELD_C;
using GermaniumProtocol::MARS_FIELD_M0;
using GermaniumProtocol::MARS_FIELD_SAUX;
using GermaniumProtocol::MARS_CH_CHEN;
using GermaniumProtocol::MARS_CH_TSEN;
using GermaniumProtocol::MARS_CH_THTR;
using GermaniumProtocol::MARS_CH_PUTR;
using GermaniumProtocol::MARS_CH_SEL;

//===========================================================================//

class Mars
{
public:
    explicit Mars( Register& reg
                 , const Logger& logger
                 , int active_channels
                 );

    void init_defaults();

    /**
     * @brief Set a global (per-chip) configuration field.
     * @param chip_mask 12-bit mask, one bit per chip.
     * @param field_id  MarsGlobalField enum value.
     * @param value     New field value.
     */
    void set_global_field( uint16_t chip_mask
                         , uint16_t field_id
                         , uint32_t value
                         );

    /**
     * @brief Set a per-channel configuration field.
     * @param channel   Channel index (0..383), or 0xFFF for all.
     * @param field_id  MarsChannelField enum value.
     * @param value     New field value.
     */
    void set_channel_field( uint16_t channel
                          , uint16_t field_id
                          , uint32_t value
                          );

    /**
     * @brief Read a global (per-chip) configuration field from cached state.
     * @param chip      Chip index (0..11).
     * @param field_id  MarsGlobalField enum value.
     * @param value     Updated with the cached field value on success.
     * @return true if chip and field_id are valid.
     */
    bool get_global_field( uint16_t chip
                         , uint16_t field_id
                         , uint32_t& value
                         ) const;

    /**
     * @brief Read a channel configuration field from cached state.
     * @param channel   Channel index (0..383).
     * @param field_id  MarsChannelField enum value.
     * @param value     Updated with the cached field value on success.
     * @return true if channel and field_id are valid.
     */
    bool get_channel_field( uint16_t channel
                          , uint16_t field_id
                          , uint32_t& value
                          ) const;

    /**
     * @brief Pack configuration and load to MARS ASICs.
     * @param chip_mask 12-bit mask selecting which chips to load.
     */
    void load( uint16_t chip_mask );

#ifdef MARS_TEST
    int test_dump_file(void); 
#endif

private:

    const int active_channels_;
    const int active_chips_;

    Register& reg_;
    const Logger& logger_;

    chipstr  globalstr_ [MAX_NCHIPS];
    chanstr  channelstr_[MAX_CHANNELS];
    uint32_t loads_     [MAX_NCHIPS][14];

    uint16_t active_chip_mask() const;
    static uint32_t reverse_bits( int nbits, uint32_t num );

    /**
     * @brief Pack chipstr/chanstr into loads[12][14] bitstream.
     */
    void wrap();

    /**
     * @brief Write packed config words to MARS_CONFIG and pulse MARS_CONF_LOAD.
     * @param chip_mask 12-bit mask selecting which chips to program.
     */
    void stuff_mars( uint16_t chip_mask );
};

//===========================================================================//
