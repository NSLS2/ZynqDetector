#include <cstring>

#include "FreeRTOS.h"

#include "Logger.hpp"
#include "Register.hpp"
#include "PsI2c.hpp"
#include "PsXadc.hpp"
//#include "Network.hpp"
#include "GermaniumNetwork.hpp"

GermaniumNetwork::GermaniumNetwork( const QueueHandle_t   register_single_access_req_queue
                                  , const QueueHandle_t   register_single_access_resp_queue
                                  , const QueueHandle_t   psi2c0_access_req_queue
                                  , const QueueHandle_t   psi2c1_access_req_queue
                                  , const QueueHandle_t   psi2c_access_resp_queue
                                  , const QueueHandle_t   psxadc_access_req_queue
                                  , const QueueHandle_t   psxadc_access_resp_queue
                                  , const QueueHandle_t   ad9252_access_req_queue
                                  , const QueueHandle_t   mars_access_req_queue
                                  , const QueueHandle_t   zddm_access_req_queue
                                  , const Logger&         logger
                                  )
                                  : Network<GermaniumNetwork>           ( logger                            )
                                  , base_                   ( static_cast<Network<GermaniumNetwork>*>(this) )
                                  , register_single_access_req_queue_   ( register_single_access_req_queue  )
                                  , register_single_access_resp_queue_  ( register_single_access_resp_queue )
                                  , psi2c0_access_req_queue_            ( psi2c0_access_req_queue           )
                                  , psi2c1_access_req_queue_            ( psi2c1_access_req_queue           )
                                  , psi2c_access_resp_queue_            ( psi2c_access_resp_queue           )
                                  , psxadc_access_req_queue_            ( psxadc_access_req_queue           )
                                  , psxadc_access_resp_queue_           ( psxadc_access_resp_queue          )
                                  , ad9252_access_req_queue_            ( ad9252_access_req_queue           )
                                  , mars_access_req_queue_              ( mars_access_req_queue             )
                                  , zddm_access_req_queue_              ( zddm_access_req_queue             )
                                  , logger_                             ( logger                            )
{
    rx_msg_map_init();
}


//===============================================================
// Initialize Rx message process handling map.
//===============================================================
void GermaniumNetwork::rx_msg_map_init()
{
    // Register single access
    this->rx_instr_map_[EVENT_FIFO_CTRL] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[DETECTOR_TYPE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[MARS_RDOUT_ENB]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[TRIG]            = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[FRAME_NO]        = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[COUNT_TIME_LO]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[COUNT_TIME_HI]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[MARS_CONF_LOAD]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[ADC_SPI]         = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[VERSIONREG]      = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[MARS_PIPE_DELAY] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[TD_CAL]          = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[COUNT_MODE]      = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[CALPULSE_RATE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[CALPULSE_WIDTH]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[CALPULSE_CNT]    = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[MARS_CALPULSE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[CALPULSE_MODE]   = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[UDP_IP_ADDR]     = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[EVENT_FIFO_CNT]  = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    this->rx_instr_map_[EVENT_FIFO_DATA] = [this]( const UdpRxMsg& msg ) { proc_register_single_access_msg( msg ); };

    // Sensor components access
    this->rx_instr_map_[STUFF_MARS]      = [this]( const UdpRxMsg& msg ) { proc_mars_access_msg( msg ); };
    this->rx_instr_map_[ADC_CLK_SKEW]    = [this]( const UdpRxMsg& msg ) { proc_ad9252_access_msg( msg ); };
    this->rx_instr_map_[ZDDM_ARM]        = [this]( const UdpRxMsg& msg ) { proc_zddm_access_msg( msg ); };

    // Peripheral access
    this->rx_instr_map_[HV]              = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    this->rx_instr_map_[HV_CUR]          = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    this->rx_instr_map_[TEMP1]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    this->rx_instr_map_[TEMP2]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    this->rx_instr_map_[TEMP3]           = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    this->rx_instr_map_[DAC_INT_REF]     = [this]( const UdpRxMsg& msg ) { proc_psi2c_access_msg( msg ); };

    // XADC access
    this->rx_instr_map_[ZTEMP]          = [this]( const UdpRxMsg& msg ) { proc_psxadc_access_msg( msg ); };
}

void GermaniumNetwork::register_i2c_handlers( const std::map<uint16_t, I2cAccessHandler>& handlers)
{
    for ( const auto& [op, fn] : handlers )
    {
        auto [it, inserted] = i2c_access_dispatch_map_.emplace(op, fn);
        if (!inserted) {
            logger_.log_error( "Duplicate opcode ", op, " detected" );
        }
    }
}



//===============================================================
// Process register single access message.
//===============================================================
void GermaniumNetwork::proc_register_single_access_msg( const UdpRxMsg& msg )
{
    RegisterSingleAccessReq req;
    req.op = msg.op;
    req.data = msg.payload.single_word.data;

    xQueueSend( register_single_access_req_queue_
              , &req
              , 0UL
              );
}
//===============================================================



//===============================================================
// Process AD9252 access message.
//===============================================================
void GermaniumNetwork::proc_ad9252_access_msg( const UdpRxMsg& msg )
{
    Ad9252AccessReq req;
    req.op       = msg.op;
    req.chip_num = msg.payload.ad9252_clk_skew.chip_num;
    req.data     = msg.payload.ad9252_clk_skew.skew;

    xQueueSend( ad9252_access_req_queue_
              , &req
              , 0UL
              );
}
//===============================================================

//===============================================================
// Process MARS access message.
//===============================================================
void GermaniumNetwork::proc_mars_access_msg( const UdpRxMsg& msg )
{
    MarsAccessReq req;
    std::memcpy( req.loads, msg.payload.stuff_mars.loads, sizeof(req.loads) );

    xQueueSend( mars_access_req_queue_
              , &req
              , 0UL
              );
}
//===============================================================

//===============================================================
// Process ZDDM access message.
//===============================================================
void GermaniumNetwork::proc_zddm_access_msg( const UdpRxMsg& msg )
{
    ZddmAccessReq req;
    req.mode = msg.payload.zddm_arm.mode;
    req.val  = msg.payload.zddm_arm.val;

    xQueueSend( zddm_access_req_queue_
              , &req
              , 0UL
              );
}
//===============================================================



//===============================================================
// Process I2C bus access message.
//===============================================================
void GermaniumNetwork::proc_psi2c_access_msg( const UdpRxMsg& msg )
{
    auto it = i2c_access_dispatch_map_.find( msg.op );

    if ( it != i2c_access_dispatch_map_.end() )
    {
        it->second( msg );
    }
    else
    {
        logger_.log_error( "Unhandled opcode: ", msg.op );
    }
}
//===============================================================

//===============================================================
// Process XADC access message.
//===============================================================
void GermaniumNetwork::proc_psxadc_access_msg( const UdpRxMsg& msg )
{
    PsXadcAccessReq req;

    req.op = msg.op;

    switch( req.op )
    {
        case ZTEMP:
            break;

        default:
            ;
    }

    xQueueSend( psxadc_access_req_queue_
              , &req
              , 0UL
              );
}
//===============================================================


//===============================================================
// Compose Tx message.
//===============================================================
size_t GermaniumNetwork::tx_msg_proc( UdpTxMsg& msg )
{
    QueueSetHandle_t resp_queue_set;
    resp_queue_set = xQueueCreateSet(50);

    xQueueAddToSet( register_single_access_resp_queue_, resp_queue_set );
    xQueueAddToSet( psi2c_access_resp_queue_, resp_queue_set );
    xQueueAddToSet( psxadc_access_resp_queue_, resp_queue_set );

    QueueSetMemberHandle_t active_queue;

    //uint16_t msg_leng;

    active_queue = (QueueHandle_t)xQueueSelectFromSet(resp_queue_set, portMAX_DELAY);

    if ( active_queue == register_single_access_resp_queue_ )
    {
        RegisterSingleAccessResp register_single_access_resp;

        xQueueReceive( register_single_access_resp_queue_, &register_single_access_resp, 0);
        msg.op = register_single_access_resp.op;
        msg.payload.single_word.data = register_single_access_resp.data;
        
        return ( 4 + sizeof(msg.payload.single_word) );
    }

    if ( active_queue == psi2c_access_resp_queue_ )
    {
        PsI2cAccessResp psi2c_access_resp;

        xQueueReceive( psi2c_access_resp_queue_, &psi2c_access_resp, 0 );
        msg.op = psi2c_access_resp.op;
        msg.payload.psi2c.length = psi2c_access_resp.length;
        memcpy( msg.payload.psi2c.data, psi2c_access_resp.data, sizeof(psi2c_access_resp.data) );
        
        return ( 4 + sizeof(msg.payload.psi2c) );
    }

    if ( active_queue == psxadc_access_resp_queue_ )
    {
        PsXadcAccessResp psxadc_access_resp;

        xQueueReceive( psxadc_access_resp_queue_, &psxadc_access_resp, 0 );
        msg.op = psxadc_access_resp.op;
        msg.payload.psxadc.length = psxadc_access_resp.length;
        memcpy( msg.payload.psxadc.data, psxadc_access_resp.data, sizeof(psxadc_access_resp.data) );

        return ( 4 + sizeof(msg.payload.psxadc) );
    }
    return 0;
}
//===============================================================
