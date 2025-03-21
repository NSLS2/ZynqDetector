#include "FreeRTOS.h"

//#include "Register.hpp"
#include "GermaniumRegister.hpp"
#include "PSI2C.hpp"
#include "PSXADC.hpp"
//#include "Network.hpp"
#include "GermaniumNetwork.hpp"


//===============================================================
// Initialize Rx message process handling map.
//===============================================================
void GermaniumNetwork::rx_msg_map_init()
{
    // Register single access
    rx_msg_map_[GermaniumRegister::EVENT_FIFO_CTRL] = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::DETECTOR_TYPE]   = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::MARS_RDOUT_ENB]  = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::TRIG]            = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::FRAME_NO]        = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::COUNT_TIME_LO]   = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::COUNT_TIME_HI]   = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::MARS_CONF_LOAD]  = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::ADC_SPI]         = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::VERSIONREG]      = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::MARS_PIPE_DELAY] = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::TD_CAL]          = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::COUNT_MODE]      = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::CALPULSE_RATE]   = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::CALPULSE_WIDTH]  = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::CALPULSE_CNT]    = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::MARS_CALPULSE]   = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::CALPULSE_MODE]   = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::UDP_IP_ADDR]     = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::EVENT_FIFO_CNT]  = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::EVENT_FIFO_DATA] = [this]( const UDPRxMsg& msg ) { proc_register_single_access_msg( msg ); };

    // Local variable update
    rx_msg_map_[GermaniumRegister::UPDATE_LOADS]    = [this]( const UDPRxMsg& msg ) { proc_update_loads_msg( msg.payload ); };

    // Register multi access
    rx_msg_map_[GermaniumRegister::STUFF_MARS]      = [this]( const UDPRxMsg& msg ) { proc_register_multi_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::AD9252_CNFG]     = [this]( const UDPRxMsg& msg ) { proc_register_multi_access_msg( msg ); };
    rx_msg_map_[GermaniumRegister::ZDDM_ARM]        = [this]( const UDPRxMsg& msg ) { proc_register_multi_access_msg( msg ); };

    // Peripheral access
    rx_msg_map_[GermaniumRegister::HV]              = [this]( const UDPRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_msg_map_[GermaniumRegister::HV_CUR]          = [this]( const UDPRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_msg_map_[GermaniumRegister::TEMP1]           = [this]( const UDPRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_msg_map_[GermaniumRegister::TEMP2]           = [this]( const UDPRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
    //rx_msg_map_[GermaniumRegister::TEMP3]           = [this]( const UDPRxMsg& msg ) { proc_psi2c_access_msg( msg ); };

    // XADC access
    //rx_msg_map_[GermaniumRegister::TEMP]          = [this]( const UDPRxMsg& msg ) { proc_psi2c_access_msg( msg ); };
}


//===============================================================
// Compose Tx message.
//===============================================================
void GermaniumNetwork::tx_msg_proc( const UDPTxMsg &msg )
{
    QueueSetMemberHandle_t active_queue_;

    RegisterAccessReq reg_acc_resp;
    PSI2CResp psi2c_resp;
    PSXADCResp psxadc_resp;

    uint16_t msg_leng;

    activeQueue = (QueueHandle_t)xQueueSelectFromSet(owner_.resp_queue_set_, portMAX_DELAY);

    switch (active)
    {
        case owner_.reg_single_access_resp_queue_:
            xQueueReceive(owner_.reg_access_resp_queue_, &reg_single_acc_resp, 0);
            msg.op = reg_acc_resp.op;
            msg.reg_acc_resp_data = reg_acc_resp.data;
            break;

        case owner_.reg_multi_access_resp_queue_:
            xQueueReceive(owner_.reg_multi_access_resp_queue_, &reg_single_acc_resp, 0);
            msg.op = reg_acc_resp.op;
            msg.reg_acc_resp_data = reg_ac_resp.data;
            
        case owner_.psi2c_resp_queue:
            xQueueReceive(owner_.psi2c_resp_queue, &psi2c_resp, 0);
            msg.op = psi2c_resp.op;
            msg.i2c_acc_resp_data = psi2c_resp.data;
            break;

        case owner_.psxadc_resp_queue:
            xQueueReceive(owner_.psxadc_resp_queue, &psxadc_resp, 0);
            msg.op = psxadc_resp.op;
            msg.xadc_acc_resp_data = psxadc_resp.data;
            break;

        default:

            break;
    }
}//===============================================================


//===============================================================
// Process register single access message.
//===============================================================
void GermaniumNetwork::proc_register_single_access_msg( const UDPRxMsg& msg )
{
    RegisterSingleAccessRequest req;
    req.op = msg.op;
    req.data = *(static_cast<uint32_t*>(msg.payload));

    xQueueSend( register_single_access_request_queue
              , req,
              , 0UL );
}
//===============================================================

//===============================================================


//===============================================================
// Process register multi access message.
//===============================================================
void GermaniumNetwork::proc_register_multi_access_msg( const UDPRxMsg& msg )
{
    RegisterMultiAccessRequest req;
    req.op = msg.op;
    switch( req.op )
    {
        case AD9252_CNFG:
            data_leng = sizeof( AD9252Cfg );
        case ZDDM_ARM:
            data_leng = sizeof( ZDDDMArm );
            memcpy( req.data, msg.payload, data_leng );
            break;
        default:
            ;
    }
    

    xQueueSend( register_single_access_request_queue
              , req,
              , 0UL );
}
//===============================================================


//===============================================================
// Updata loads.
//===============================================================
void GermaniumNetwork::proc_update_loads_msg( char* loads )
{
    owner_.update_loads( loads );
}
