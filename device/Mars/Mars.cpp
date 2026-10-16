/**
 * @file Mars.cpp
 * @brief MARS ASIC configuration — Linux version.
 * @details
 * Non-template implementation. Uses Register::multi_access for
 * atomic chip-level register sequences in stuff_mars().
 *
 * Ported from germ-zmq-server MarsConfig + ZynqDetector Mars.tpp.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "Mars.hpp"

#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>
#include <iostream>
#ifdef MARS_TEST
#include <cerrno>
#include <cstring>
#include <sys/stat.h>
#include <fstream>
#endif

//===========================================================================//

Mars::Mars( Register&     reg
          , const Logger& logger
          , int           active_channels
          )
          : reg_             ( reg                                 )
          , logger_          ( logger                              )
          , active_channels_ ( active_channels                     )
          , active_chips_    ( active_channels / CHANNELS_PER_CHIP )
{
    std::memset( globalstr_,  0, sizeof(globalstr_)  );
    std::memset( channelstr_, 0, sizeof(channelstr_) );
    std::memset( loads_,      0, sizeof(loads_)      );
}

//===========================================================================//

uint16_t Mars::active_chip_mask() const
{
    return static_cast<uint16_t>((1u << active_chips_) - 1u);
}

//===========================================================================//

void Mars::init_defaults()
{
    for ( int chip = 0; chip < active_chips_; chip++ )
    {
        globalstr_[chip].pa     = 380;
        globalstr_[chip].pb     = 102;
        globalstr_[chip].rm     = 1;
        globalstr_[chip].senfl2 = 1;
        globalstr_[chip].sbn    = 1;
        globalstr_[chip].sb     = 1;
        globalstr_[chip].sp     = 1;
        globalstr_[chip].sbm    = 1;
    }

    for ( int ch = 0; ch < active_channels_; ch++ )
    {
        channelstr_[ch].dp  = 7;
        channelstr_[ch].da  = 3;
        channelstr_[ch].sel = 1;
    }
}

//===========================================================================//

void Mars::set_global_field( uint16_t chip_mask
                           , uint16_t field_id
                           , uint32_t value
                           )
{
    for ( int chip = 0; chip < active_chips_; chip++ )
    {
        if ( !(chip_mask & (1 << chip)) )
            continue;

        switch ( field_id )
        {
            case MARS_FIELD_ST:   globalstr_[chip].ts   = value; break;
            case MARS_FIELD_GAIN:
                logger_.log_warn("MARS TEST: set gain chip=%d old=%u new=%u chip_mask=0x%03x",
                                 chip,
                                 static_cast<unsigned>(globalstr_[chip].g),
                                 static_cast<unsigned>(value),
                                 chip_mask);

                globalstr_[chip].g = value;
                break;
            case MARS_FIELD_POL:  globalstr_[chip].sp   = value; break;
            case MARS_FIELD_EBLK:
                switch ( value )
                {
                    case 0: // Off
                        globalstr_[chip].sl  = 1;
                        globalstr_[chip].slh = 0;
                        break;
          
                    case 1: // 2pA
                        globalstr_[chip].sl  = 0;
                        globalstr_[chip].slh = 0;
                        break;
          
                    case 2: // 8pA
                        globalstr_[chip].sl  = 0;
                        globalstr_[chip].slh = 1;
                        break;
          
                    default:
                        logger_.log_warn("MARS: EBLK value %u out of range", value);
                        break;
                }
                break;

            case MARS_FIELD_GMON: globalstr_[chip].m1   = value; break;
            case MARS_FIELD_PUEN: globalstr_[chip].spur = value; break;
            case MARS_FIELD_MFS:  globalstr_[chip].sse  = value; break;
            case MARS_FIELD_TDS:
	    {
	        uint8_t tr = 0;
		uint8_t rt = 0;

		switch ( value )
		{
                    case 0: tr = 0; rt = 0; break;
                    case 1: tr = 1; rt = 0; break;
                    case 2: tr = 2; rt = 0; break;
                    case 3: tr = 3; rt = 0; break;
                    case 4: tr = 1; rt = 1; break;
                    case 5: tr = 2; rt = 1; break;
                    case 6: tr = 3; rt = 1; break;
                    default:
                        logger_.log_warn("MARS: TDS value %u out of range", value);
                        break;
		}
		globalstr_[chip].tr = tr;
      		globalstr_[chip].rt = rt;
		break;
	    }
            case MARS_FIELD_TDM:   globalstr_[chip].tm   = value; break;
            case MARS_FIELD_TH:    globalstr_[chip].pa   = value; break;
	    case MARS_FIELD_TPAMP: globalstr_[chip].pb   = value; break;
            case MARS_FIELD_C:     globalstr_[chip].c    = value; break;
            case MARS_FIELD_M0:    globalstr_[chip].m0   = value; break;
            case MARS_FIELD_SAUX:  globalstr_[chip].saux = value; break;
            default:
                logger_.log_warn("MARS: unknown global field_id %d", field_id);
                break;
        }
    }
}

//===========================================================================//

void Mars::set_channel_field( uint16_t channel
                            , uint16_t field_id
                            , uint32_t value
                            )
{
    int start = 0;
    int end   = active_channels_;

    if ( channel != 0xFFF )
    {
        if ( channel >= active_channels_ )
        {
            logger_.log_warn("MARS: channel %d out of range", channel);
            return;
        }
        start = channel;
        end   = channel + 1;
    }

    for ( int ch = start; ch < end; ch++ )
    {
        switch ( field_id )
        {
            case MARS_CH_CHEN:
                channelstr_[ch].sm = value ? 0 : 1;
                break;
            case MARS_CH_TSEN:
                channelstr_[ch].st = value;
                break;
            case MARS_CH_THTR:
                channelstr_[ch].da = value;
                break;
            case MARS_CH_PUTR:
                channelstr_[ch].dp = value;
                break;
            case MARS_CH_SEL:
                channelstr_[ch].sel = value ? 1 : 0;
                break;
            default:
                logger_.log_warn("MARS: unknown channel field_id %d", field_id);
                break;
        }
    }
}

//===========================================================================//

bool Mars::get_global_field( uint16_t chip
                           , uint16_t field_id
                           , uint32_t& value
                           ) const
{
    if ( chip >= active_chips_ )
    {
        logger_.log_warn("MARS: chip %d out of range", chip);
        return false;
    }

    const chipstr& g = globalstr_[chip];

    switch ( field_id )
    {
        case MARS_FIELD_ST:   value = g.ts;   return true;
        case MARS_FIELD_GAIN: value = g.g;    return true;
        case MARS_FIELD_POL:  value = g.sp;   return true;
        case MARS_FIELD_EBLK:
            if ( g.sl )
                value = 0;
            else if ( g.slh )
                value = 2;
            else
                value = 1;
            return true;

        case MARS_FIELD_GMON: value = g.m1;   return true;
        case MARS_FIELD_PUEN: value = g.spur; return true;
        case MARS_FIELD_MFS:  value = g.sse;  return true;
        case MARS_FIELD_TDS:  value = g.tr;   return true;
            if ( g.rt == 0 )
            {
                value = g.tr & 0x3;
                return true;
            }
      
            switch ( g.tr & 0x3 )
            {
                case 1: value = 4; return true;
                case 2: value = 5; return true;
                case 3: value = 6; return true;
                default:
                    value = 0;
                    return true;
            }

        case MARS_FIELD_TDM:   value = g.tm;   return true;
        case MARS_FIELD_TH:    value = g.pa;   return true;
        case MARS_FIELD_TPAMP: value = g.pb;   return true;
        case MARS_FIELD_C:     value = g.c;    return true;
        case MARS_FIELD_M0:    value = g.m0;   return true;
        case MARS_FIELD_SAUX:  value = g.saux; return true;
        default:
            logger_.log_warn("MARS: unknown global field_id %d", field_id);
            return false;
    }
}

//===========================================================================//

bool Mars::get_channel_field( uint16_t channel
                            , uint16_t field_id
                            , uint32_t& value
                            ) const
{
    if ( channel >= active_channels_ )
    {
        logger_.log_warn("MARS: channel %d out of range", channel);
        return false;
    }

    const chanstr& ch = channelstr_[channel];

    switch ( field_id )
    {
        case MARS_CH_CHEN: value = ch.sm ? 0 : 1; return true;
        case MARS_CH_TSEN: value = ch.st;         return true;
        case MARS_CH_THTR: value = ch.da;         return true;
        case MARS_CH_PUTR: value = ch.dp;         return true;
    	case MARS_CH_SEL:  value = ch.sel;        return true;
        default:
            logger_.log_warn("MARS: unknown channel field_id %d", field_id);
            return false;
    }
}

//===========================================================================//

void Mars::load( uint16_t chip_mask )
{
    chip_mask &= active_chip_mask();
    
    if ( chip_mask == 0 )
    {
        logger_.log_warn("MARS: load requested with no active chips");
        return;
    }

    wrap();

#ifdef MARS_TEST
    test_dump_file();
#endif

    stuff_mars( chip_mask );
}

//===========================================================================//
// Test MARS driver.

#ifdef MARS_TEST

int Mars::test_dump_file( void )
{
    static const char* path = "/tmp/mars-test/mars_zynq_dump.csv";
    static bool dir_created = false;

    std::cerr << __func__ << ": dumping MARS configuration file...\n";

    if ( !dir_created )
    {
        if ( (mkdir("/tmp/mars-test", 0775) == -1) &&
	     ( errno != EEXIST) )
	{
	    std::cerr << __func__ << ": failed to create /tmp/mars-test\n";
            return -1;
	}
	dir_created = true;
    }

    std::ofstream fp(path);
    if ( !fp.is_open() )
    {
        std::printf("MARS_TEST_ERROR cannot open %s\n", path);
        return -1;
    }

    fp << "TYPE,INDEX,CHIP,CHN,FIELD,VALUE\n";

    for ( int chip = 0; chip < active_chips_; chip++ )
    {
        const chipstr& g = globalstr_[chip];

        fp << "GLOBAL," << chip << "," << chip << ",,pa," << g.pa << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,pb," << g.pb << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,rm," << static_cast<unsigned>(g.rm) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,senfl1," << static_cast<unsigned>(g.senfl1) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,senfl2," << static_cast<unsigned>(g.senfl2) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,m0," << static_cast<unsigned>(g.m0) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,m1," << static_cast<unsigned>(g.m1) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,sbn," << static_cast<unsigned>(g.sbn) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,sb," << static_cast<unsigned>(g.sb) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,sl," << static_cast<unsigned>(g.sl) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,ts," << static_cast<unsigned>(g.ts) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,rt," << static_cast<unsigned>(g.rt) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,spur," << static_cast<unsigned>(g.spur) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,sse," << static_cast<unsigned>(g.sse) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,tr," << static_cast<unsigned>(g.tr) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,ss," << static_cast<unsigned>(g.ss) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,c," << static_cast<unsigned>(g.c) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,g," << static_cast<unsigned>(g.g) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,slh," << static_cast<unsigned>(g.slh) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,sp," << static_cast<unsigned>(g.sp) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,saux," << static_cast<unsigned>(g.saux) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,sbm," << static_cast<unsigned>(g.sbm) << "\n";
        fp << "GLOBAL," << chip << "," << chip << ",,tm," << static_cast<unsigned>(g.tm) << "\n";
    }

    for ( int channel = 0; channel < active_channels_; channel++ )
    {
        const chanstr& c = channelstr_[channel];
        int chip = channel / CHANNELS_PER_CHIP;
        int chn  = channel % CHANNELS_PER_CHIP;

        fp << "CHANNEL," << channel << "," << chip << "," << chn << ",dp," << static_cast<unsigned>(c.dp) << "\n";
        fp << "CHANNEL," << channel << "," << chip << "," << chn << ",nc1," << static_cast<unsigned>(c.nc1) << "\n";
        fp << "CHANNEL," << channel << "," << chip << "," << chn << ",da," << static_cast<unsigned>(c.da) << "\n";
        fp << "CHANNEL," << channel << "," << chip << "," << chn << ",sel," << static_cast<unsigned>(c.sel) << "\n";
        fp << "CHANNEL," << channel << "," << chip << "," << chn << ",nc2," << static_cast<unsigned>(c.nc2) << "\n";
        fp << "CHANNEL," << channel << "," << chip << "," << chn << ",sm," << static_cast<unsigned>(c.sm) << "\n";
        fp << "CHANNEL," << channel << "," << chip << "," << chn << ",st," << static_cast<unsigned>(c.st) << "\n";
    }

    std::ios_base::fmtflags flags = fp.flags();

    for ( int chip = 0; chip < active_chips_; chip++ )
    {
        for ( int word = 0; word < 14; word++ )
        {
            fp << "LOAD," << chip << "," << chip << ",,word" << word << ",";
            fp << std::hex << std::nouppercase;
            fp.width(8);
            fp.fill('0');
            fp << loads_[chip][word];
            fp.flags(flags);
            fp << "\n";
        }
    }

    return 0;
}

#endif

//===========================================================================//

uint32_t Mars::reverse_bits( int nbits, uint32_t num )
{
    //uint32_t reversed = 0;
    //for ( int i = 0; i < nbits; i++ )
    //{
    //    if ( num & (1 << i) )
    //        reversed |= 1 << ( (nbits - 1) - i );
    //}
    //return reversed;
    (void)nbits;
    return num;
}

//===========================================================================//

namespace
{

class BitPacker
{
public:
    explicit BitPacker( uint32_t* words )
        : words_( words ), pos_( 0 )
    {
        std::memset( words, 0, 14 * sizeof(uint32_t) );
    }

    void push( uint32_t val, int nbits )
    {
        for ( int i = nbits - 1; i >= 0; i-- )
        {
            if ( val & (1u << i) )
                words_[pos_ / 32] |= (1u << (31 - (pos_ % 32)));
            pos_++;
        }
    }

    void pad( int nbits ) { pos_ += nbits; }

private:
    uint32_t* words_;
    int       pos_;
};

} // anonymous namespace

//===========================================================================//

void Mars::wrap()
{
    for ( int chip = 0; chip < active_chips_; chip++ )
    {
        BitPacker bp( loads_[chip] );
        const chipstr& g = globalstr_[chip];

        //---------------------------------------------------------------
        // Global configuration — 49 bits
        //---------------------------------------------------------------

        bp.push( g.tm,     1 );
        bp.push( g.sbm,    1 );
        bp.push( g.saux,   1 );
        bp.push( g.sp,     1 );
        bp.push( g.slh,    1 );
        bp.push( reverse_bits( 2, g.g ),  2 );
        bp.push( reverse_bits( 5, g.c ),  5 );
        bp.push( reverse_bits( 2, g.ss ), 2 );
        bp.push( reverse_bits( 2, g.tr ), 2 );
        bp.push( g.sse,    1 );
        bp.push( g.spur,   1 );
        bp.push( g.rt,     1 );
        bp.push( reverse_bits( 2, g.ts ), 2 );
        bp.push( g.sl,     1 );
        bp.push( g.sb,     1 );
        bp.push( g.sbn,    1 );
        bp.push( g.m1,     1 );
        bp.push( g.m0,     1 );
        bp.push( g.senfl2, 1 );
        bp.push( g.senfl1, 1 );
        bp.push( g.rm,     1 );
        bp.push( reverse_bits( 10, g.pb ), 10 );
        bp.push( reverse_bits( 10, g.pa ), 10 );
        // pos = 49

        //---------------------------------------------------------------
        // Channel configuration — 384 bits (32 channels × 12 bits)
        // Channels packed in reverse order: 31 down to 0.
        //---------------------------------------------------------------

        for ( int ch = 31; ch >= 0; ch-- )
        {
            const chanstr& c = channelstr_[chip * CHANNELS_PER_CHIP + ch];

            bp.push( c.st,                       1 );
            bp.push( c.sm,                       1 );
            bp.push( c.nc2,                      1 );
            bp.push( c.sel,                      1 );
            bp.push( reverse_bits( 3, c.da ),    3 );
            bp.push( c.nc1,                      1 );
            bp.push( reverse_bits( 4, c.dp ),    4 );
        }
        // pos = 49 + 384 = 433

        //---------------------------------------------------------------
        // Padding — 15 zero bits
        //---------------------------------------------------------------

        bp.pad( 15 );
        // pos = 448 = 14 × 32
    }
}

//===========================================================================//

void Mars::stuff_mars( uint16_t chip_mask )
{
    for ( int i = 0; i < active_chips_; i++ )
    {
        if ( !(chip_mask & (1 << i)) )
            continue;

        ///< Hold register mutex for one chip at a time
        reg_.multi_access_start();

        ///< Start sequence for this chip
        reg_.multi_access_write( GermaniumProtocol::Register::MARS_CONF_LOAD, 4 );
        reg_.multi_access_write( GermaniumProtocol::Register::MARS_CONF_LOAD, 0 );

        ///< Write 14 configuration words with latch pulse between each
        for ( int j = 0; j < 14; j++ )
        {
            reg_.multi_access_write( GermaniumProtocol::Register::MARS_CONFIG, loads_[i][j] );

            ///< Latch pulse
            reg_.multi_access_write( GermaniumProtocol::Register::MARS_CONF_LOAD, 2 );
            reg_.multi_access_write( GermaniumProtocol::Register::MARS_CONF_LOAD, 0 );

            std::this_thread::sleep_for( std::chrono::milliseconds(1) );
        }

        ///< Chip select pulse
        reg_.multi_access_write( GermaniumProtocol::Register::MARS_CONF_LOAD, 0x00010000u << i );
        reg_.multi_access_write( GermaniumProtocol::Register::MARS_CONF_LOAD, 0 );

        reg_.multi_access_end();

        ///< Release between chips so register_worker_ can service reads
        std::this_thread::sleep_for( std::chrono::milliseconds(1) );
    }

    logger_.log_debug("MARS: stuff_mars complete (chip_mask=0x%03X)", chip_mask);
}

//===========================================================================//
