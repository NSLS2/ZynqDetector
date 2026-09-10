/**
 * @file GermaniumDetector.cpp
 * @brief Member function definitions of `GermaniumDetector` — async version.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "GermaniumDetector.hpp"

#include <cstdio>
#include <thread>
#include <chrono>
#include <algorithm>

//===========================================================================//
//  LTC2309 ADC channel control words
//===========================================================================//
static constexpr uint8_t LTC2309_CTRL[] = {
    0x88,  // ch0
    0xC8,  // ch1
    0x98,  // ch2
    0xD8,  // ch3
    0xA8,  // ch4
    0xE8,  // ch5
    0xB8,  // ch6
    0xF8,  // ch7
};

//===========================================================================//
//  DAC7678 constants
//===========================================================================//
static constexpr uint8_t DAC7678_ADDR = 0x4A;   ///< I2C slave address
static constexpr uint8_t LTC2309_ADDR = 0x08;   ///< I2C slave address

//===========================================================================//

GermaniumDetector::GermaniumDetector( int nelm )
    : ZynqDevice< GermaniumDetector
                , GermaniumZMQ
                , GermaniumZynq
                >()
    , active_channels_( nelm      )
    , active_chips_   ( nelm / 32 )
{
    printf("GermaniumDetector: initialization started...\n");

    create_queues();

    create_components();

    ///< Hardware init (before network starts)
    init_hardware();

    network_init();

    create_tasks();

    printf("GermaniumDetector: initialization complete\n");
    printf("=================================================\n");
}

//===========================================================================//

void GermaniumDetector::create_queues_special()
{
    ///< No additional queues — AsyncWorkers use ThreadQueue internally.
}

//===========================================================================//

void GermaniumDetector::create_components_special()
{
    auto z = std::make_unique<GermaniumZynq>( this->logger_ );
    this->set_zynq( std::move(z) );

    auto n = std::make_unique<GermaniumZMQ>( this->logger_ );
    this->set_network( std::move(n) );

    mars_ = std::make_unique<Mars>( this->zynq_->reg()
                                  , this->logger_
                                  , active_channels_
                                  );
}

//===========================================================================//

void GermaniumDetector::create_tasks_special()
{
    ///< Response callback — pushes replies to Network tx path
    auto sender = [this](const GermaniumProtocol::Message& msg) {
        this->network_->tx_reply(msg);
    };

    //--------------------------------------------------------------
    // Register worker
    //--------------------------------------------------------------
    reg_worker_ = std::make_unique<AsyncWorker>( "REG", sender, logger_ );
    reg_worker_->set_handler(
        [this](GermaniumProtocol::Message& msg) {
            auto& reg = zynq_->reg();
            if ( msg.cmd == GermaniumProtocol::Command::REG_READ ) {
                msg.value = reg.read( static_cast<uint16_t>(msg.addr) );
            } else {
                reg.write( static_cast<uint16_t>(msg.addr), msg.value );
            }
        }
    );
    reg_worker_->start();

    //--------------------------------------------------------------
    // MARS worker — stuff_mars (long-running, holds register mutex)
    //--------------------------------------------------------------
    mars_worker_ = std::make_unique<AsyncWorker>( "MARS", sender, logger_ );
    mars_worker_->set_handler(
        [this](GermaniumProtocol::Message& msg) {
            uint16_t chip_mask = static_cast<uint16_t>(msg.addr & 0xFFF);
            mars_->load( chip_mask );
        }
    );
    mars_worker_->start();

    //--------------------------------------------------------------
    // SPI worker — AD9252 clock skew
    //--------------------------------------------------------------
    spi_worker_ = std::make_unique<AsyncWorker>( "SPI", sender, logger_ );
    spi_worker_->set_handler(
        [this](GermaniumProtocol::Message& msg) {
            int chip = static_cast<int>(msg.addr);
            int skew = static_cast<int>(msg.value);
            ad9252_set_clk_skew( chip, skew );
            if ( chip >= 1 && chip <= 3 )
                adc_clk_skew_[chip] = skew;
        }
    );
    spi_worker_->start();

    //--------------------------------------------------------------
    // I2C bus 0 worker — TMP100 ×3
    //--------------------------------------------------------------
    i2c0_worker_ = std::make_unique<AsyncWorker>( "I2C0", sender, logger_ );
    i2c0_worker_->set_handler(
        [this](GermaniumProtocol::Message& msg) {
            // ADGermanium sends logical selectors; ZynqDetector owns the
            // board-specific TMP100 addresses from Mars_DDM Tmp100.db.
            static constexpr uint8_t tmp100_addrs[] = { 0x48, 0x49, 0x4A };
            if ( msg.addr > 2 ) { msg.value = 0; return; }

            uint8_t slave = tmp100_addrs[msg.addr];
            uint8_t data[2] = {0};
            int rc = zynq_->i2c0().read( slave, data, 2 );
            if ( rc < 0 ) { msg.value = 0; return; }

            msg.value = (static_cast<uint32_t>(data[0]) << 8) | data[1];
        }
    );
    i2c0_worker_->start();

    //--------------------------------------------------------------
    // I2C bus 1 worker — DAC7678 + LTC2309
    //--------------------------------------------------------------
    i2c1_worker_ = std::make_unique<AsyncWorker>( "I2C1", sender, logger_ );
    i2c1_worker_->set_handler(
        [this](GermaniumProtocol::Message& msg) {
            auto& i2c = zynq_->i2c1();

            switch ( msg.cmd )
            {
                case GermaniumProtocol::Command::I2C_DAC_WRITE:
                {
                    ///< DAC7678 write: [0x30+ch, value_MSB, value_LSB]
                    uint8_t ch   = static_cast<uint8_t>(msg.addr & 0x07);
                    uint16_t val = static_cast<uint16_t>(std::min<uint32_t>(msg.value, 4095));

                    uint8_t buf[3] = { static_cast<uint8_t>(0x30 + ch)
		                     , static_cast<uint8_t>((val & 0x0FF0) >> 4)
				     , static_cast<uint8_t>((val & 0x000F) << 4)
                                     };

                    i2c.write( DAC7678_ADDR, buf, 3 );
                    break;
                }

                case GermaniumProtocol::Command::I2C_ADC_READ:
                {
                    ///< LTC2309 read: write ctrl word, wait, read 2 bytes
                    uint8_t ch = static_cast<uint8_t>(msg.addr & 0x07);
                    uint8_t ctrl = LTC2309_CTRL[ch];
                    i2c.write( LTC2309_ADDR, &ctrl, 1 );

                    std::this_thread::sleep_for( std::chrono::milliseconds(10) );

                    uint8_t data[2] = {0};
                    i2c.read( LTC2309_ADDR, data, 2 );
                    msg.value = (static_cast<uint32_t>(data[0]) << 4)
                              | (static_cast<uint32_t>(data[1]) >> 4);
                    break;
                }

                case GermaniumProtocol::Command::I2C_DAC_INIT:
                {
                    ///< DAC7678 enable internal 2.5V reference
                    uint8_t buf[3] = { 0x80, 0x00, 0x10 };
                    i2c.write( DAC7678_ADDR, buf, 3 );
                    break;
                }

                default:
                    break;
            }
        }
    );
    i2c1_worker_->start();

    //--------------------------------------------------------------
    // XADC worker — Zynq on-chip temperature
    //--------------------------------------------------------------
    xadc_worker_ = std::make_unique<AsyncWorker>( "XADC", sender, logger_ );
    xadc_worker_->set_handler(
        [this](GermaniumProtocol::Message& msg) {
#ifdef SIM_MODE
            msg.value = 2423;  ///< ~25°C
#else
            uint32_t raw = 0;
            FILE* f = std::fopen(
                "/sys/bus/iio/devices/iio:device0/in_temp0_raw", "r" );
            if ( f ) {
                if ( std::fscanf( f, "%u", &raw ) != 1 )
                    raw = 0;
                std::fclose( f );
            }
            msg.value = raw;
#endif
        }
    );
    xadc_worker_->start();
}

//===========================================================================//
//  Hardware initialization (before network loop)
//===========================================================================//

void GermaniumDetector::init_hardware()
{
    logger_.log_debug("GermaniumDetector: initializing hardware...");

    init_dac_reference();
    init_adc_clk_skew();
    init_mars_defaults();

    ///< Set detector model and readout enable
    uint32_t det_model;
    uint32_t mars_rdout_enable;
    if ( active_channels_ == 192 )
    {
        det_model = 0;
        mars_rdout_enable = 0x8AAA;
    }
    else
    {
        det_model = 1;
        mars_rdout_enable = 0x8FFF;
    }
    zynq_->reg().write( GermaniumReg::DETECTOR_MODEL, det_model );
    zynq_->reg().write( GermaniumReg::MARS_RDOUT_ENB, mars_rdout_enable );

    logger_.log_debug("GermaniumDetector: hardware init complete");
}

//===========================================================================//

void GermaniumDetector::init_dac_reference()
{
    ///< Enable DAC7678 internal 2.5V reference on I2C bus 1
    uint8_t buf[3] = { 0x80, 0x00, 0x10 };
    zynq_->i2c1().write( DAC7678_ADDR, buf, 3 );
    logger_.log_debug("GermaniumDetector: DAC7678 internal reference enabled");
}

//===========================================================================//

void GermaniumDetector::init_adc_clk_skew()
{
    ///< Same values as Mars_DDM zDDM_init()
    ad9252_cnfg(1, 22, 9);   ad9252_cnfg(1, 255, 1);
    ad9252_cnfg(2, 22, 8);   ad9252_cnfg(2, 255, 1);
    ad9252_cnfg(3, 22, 8);   ad9252_cnfg(3, 255, 1);
    logger_.log_debug("GermaniumDetector: ADC clock skew configured");
}

//===========================================================================//

void GermaniumDetector::init_mars_defaults()
{
    /////< Set MARS defaults matching Mars_DDM + positive polarity
    //for ( int chip = 0; chip < 12; chip++ )
    //{
    //    mars_->set_global_field( 1u << chip, MARS_FIELD_POL,  1 );  ///< positive
    //    mars_->set_global_field( 1u << chip, MARS_FIELD_GAIN, 0 );  ///< gain 0
    //    mars_->set_global_field( 1u << chip, MARS_FIELD_ST,   0 );  ///< shaping time 0
    //    mars_->set_global_field( 1u << chip, MARS_FIELD_TH,  512 ); ///< mid-scale threshold
    //}

    /////< Enable all channels, select shaper output
    //for ( int ch = 0; ch < 384; ch++ )
    //{
    //    mars_->set_channel_field( ch, MARS_CH_CHEN, 1 );   ///< sm = 0 (enabled)
    //}

    mars_->init_defaults();

    uint16_t chip_mask = static_cast<uint16_t>((1u << active_chips_) - 1u);
    mars_->load( chip_mask );
    logger_.log_debug("GermaniumDetector: MARS defaults loaded");
}

//===========================================================================//
//  Non-blocking command dispatch (called by rx_thread)
//===========================================================================//

void GermaniumDetector::dispatch_command( const GermaniumProtocol::Message& msg )
{
    switch ( msg.cmd )
    {
        case GermaniumProtocol::Command::REG_READ:
        case GermaniumProtocol::Command::REG_WRITE:
            reg_worker_->submit( msg );
            break;

        case GermaniumProtocol::Command::MARS_GLOBAL_SET:
        {
            ///< Inline — update Mars in-memory state (fast, no HW I/O)
            uint16_t chip_mask = static_cast<uint16_t>( (msg.addr >> 16) & 0xFFF );
            uint16_t field_id  = static_cast<uint16_t>( msg.addr & 0xFFFF );
            mars_->set_global_field( chip_mask, field_id, msg.value );

            network_->tx_reply( msg );  ///< echo back
            break;
        }

        case GermaniumProtocol::Command::MARS_GLOBAL_READ:
        {
            GermaniumProtocol::Message reply = msg;
            uint16_t chip_mask = static_cast<uint16_t>( (msg.addr >> 16) & 0xFFF );
            uint16_t field_id  = static_cast<uint16_t>( msg.addr & 0xFFFF );
            uint16_t chip      = 0;

            if ( chip_mask != 0 )
            {
                while ( chip < 12 && !(chip_mask & (1u << chip)) )
                    chip++;
            }

            if ( !mars_->get_global_field( chip, field_id, reply.value ) )
                reply.value = 0;

            network_->tx_reply( reply );
            break;
        }

        case GermaniumProtocol::Command::MARS_CHANNEL_SET:
        {
            ///< Inline — update Mars in-memory state (fast, no HW I/O)
            uint16_t channel  = static_cast<uint16_t>( (msg.addr >> 16) & 0xFFF );
            uint16_t field_id = static_cast<uint16_t>( msg.addr & 0xFFFF );
            mars_->set_channel_field( channel, field_id, msg.value );

            network_->tx_reply( msg );  ///< echo back
            break;
        }

        case GermaniumProtocol::Command::MARS_CHANNEL_READ:
        {
            GermaniumProtocol::Message reply = msg;
            uint16_t channel  = static_cast<uint16_t>( (msg.addr >> 16) & 0xFFF );
            uint16_t field_id = static_cast<uint16_t>( msg.addr & 0xFFFF );

            if ( !mars_->get_channel_field( channel, field_id, reply.value ) )
                reply.value = 0;

            network_->tx_reply( reply );
            break;
        }

        case GermaniumProtocol::Command::MARS_LOAD:
            mars_worker_->submit( msg );
            break;

        case GermaniumProtocol::Command::ADC_CLK_SKEW_SET:
            spi_worker_->submit( msg );
            break;

        case GermaniumProtocol::Command::ADC_CLK_SKEW_READ:
        {
            GermaniumProtocol::Message reply = msg;
            int chip = static_cast<int>(msg.addr);
            reply.value = (chip >= 1 && chip <= 3)
                        ? static_cast<uint32_t>(adc_clk_skew_[chip]) : 0;
            network_->tx_reply( reply );
            break;
        }

        case GermaniumProtocol::Command::I2C_TEMP_READ:
            i2c0_worker_->submit( msg );
            break;

        case GermaniumProtocol::Command::XADC_READ:
            xadc_worker_->submit( msg );
            break;

        case GermaniumProtocol::Command::I2C_DAC_WRITE:
        case GermaniumProtocol::Command::I2C_ADC_READ:
        case GermaniumProtocol::Command::I2C_DAC_INIT:
            i2c1_worker_->submit( msg );
            break;

        case GermaniumProtocol::Command::SET_LOG_LEVEL:
        {
            logger_.set_log_control( static_cast<uint8_t>(msg.value) );
            logger_.log_debug( "GermaniumDetector: log level set to 0x%02X",
                               logger_.read_log_control() );
            network_->tx_reply( msg );
            break;
        }

        case GermaniumProtocol::Command::GET_PROTOCOL_VERSION:
        {
            GermaniumProtocol::Message reply = msg;
            reply.addr = 0;
            reply.value = GermaniumProtocol::PROTOCOL_VERSION;
            network_->tx_reply( reply );
            break;
        }

        case GermaniumProtocol::Command::HEARTBEAT:
            network_->tx_reply( msg );
            break;

        default:
            logger_.log_warn( "GermaniumDetector: unknown cmd 0x%02X", msg.cmd );
            ///< Still send a reply so the IOC doesn't hang
            network_->tx_reply( msg );
            break;
    }
}

//===========================================================================//
//  AD9252 SPI bit-bang (unchanged from linux-zmq)
//===========================================================================//

void GermaniumDetector::ad9252_set_clk_skew( int chip_num, int skew )
{
    logger_.log_debug( "[%s]: chip=%d, skew=%d", __func__, chip_num, skew );
    ad9252_cnfg( chip_num, 22, skew );
    ad9252_cnfg( chip_num, 255, 1 );
}

//===========================================================================//

void GermaniumDetector::ad9252_cnfg( int chip_num, int addr, int val )
{
    int chip_sel;

    switch ( chip_num )
    {
        case 1:  chip_sel = 0b11000; break;
        case 2:  chip_sel = 0b10100; break;
        case 3:  chip_sel = 0b01100; break;
        default: chip_sel = 0b00000; break;
    }

    auto& reg = zynq_->reg();

    reg.write( GermaniumReg::ADC_SPI, chip_sel );
    ad9252_load_reg( chip_sel, addr, val );
    reg.write( GermaniumReg::ADC_SPI, 0b11100 );
}

//===========================================================================//

void GermaniumDetector::ad9252_load_reg( int chip_sel, int addr, int data )
{
    int i;

    for ( i = 0; i < 100; i++ )
        ;

    auto& reg = zynq_->reg();

    reg.multi_access_start();

    ad9252_send_spi_bit( chip_sel, 0 );

    for ( i = 1; i >= 0; i-- )
        ad9252_send_spi_bit( chip_sel, 0 );

    for ( i = 12; i >= 0; i-- )
        ad9252_send_spi_bit( chip_sel, addr >> i );

    for ( i = 7; i >= 0; i-- )
        ad9252_send_spi_bit( chip_sel, data >> i );

    reg.multi_access_end();

    for ( i = 0; i < 100; i++ )
        ;
}

//===========================================================================//

void GermaniumDetector::ad9252_send_spi_bit( int chip_sel, int val )
{
    int sda = val & 0x1;

    auto& reg = zynq_->reg();

    reg.multi_access_write( GermaniumReg::ADC_SPI, (chip_sel | 0) );
    reg.multi_access_write( GermaniumReg::ADC_SPI, (chip_sel | sda) );
    reg.multi_access_write( GermaniumReg::ADC_SPI, (chip_sel | 0x2 | sda) );
    reg.multi_access_write( GermaniumReg::ADC_SPI, (chip_sel | sda) );
    reg.multi_access_write( GermaniumReg::ADC_SPI, (chip_sel | 0) );
}

//===========================================================================//
