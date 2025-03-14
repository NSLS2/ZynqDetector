// C++ includes
#include <iterator>
#include <portmacro.h>
#include <memory>
// FreeRTOS includes
#include "FreeRTOS.h"
#include "msg.hpp"
#include "task.h"
#include "queue.h"
#include "timers.h"
// Xilinx includes
#include "xil_printf.h"
#include "xparameters.h"
// Project includes
#include "zynq_detector.hpp"
#include "pynq_ssd_msg.hpp"

#define TIMER_ID	1
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define TIMER_CHECK_THRESHOLD	9
/*-----------------------------------------------------------*/

/* The Tx and Rx tasks as described at the top of this file. */
/*-----------------------------------------------------------*/

/* The queue used by the Tx and Rx tasks, as described at the top of this
file. */
static TaskHandle_t xTxTask;
static TaskHandle_t xRxTask;
static QueueHandle_t xQueue = NULL;
static TimerHandle_t xPollTimer = NULL;
char HWstring[15] = "Hello World";
long RxtaskCntr = 0;

#if (configSUPPORT_STATIC_ALLOCATION == 1)
#define QUEUE_BUFFER_SIZE		100

uint8_t ucQueueStorageArea[ QUEUE_BUFFER_SIZE ];
StackType_t xStack1[ configMINIMAL_STACK_SIZE ];
StackType_t xStack2[ configMINIMAL_STACK_SIZE ];
StaticTask_t xTxBuffer,xRxBuffer;
StaticTimer_t xTimerBuffer;
static StaticQueue_t xStaticQueue;
#endif

GeDetector::GeDetector()
{
    instr_map_[RD_EVENT_FIFO_CTRL] = [this](udp_rx_msg_t msg){ proc_event_fifo_ctrl( msg ); };
    instr_map_[WR_EVENT_FIFO_CTRL] = [this](udp_rx_msg_t msg){ proc_event_fifo_ctrl( msg ); };
    instr_map_[RD_DETECTOR_TYPE]   = [this](udp_rx_msg_t msg){ proc_detector_type( msg ); };
    instr_map_[WR_DETECTOR_TYPE]   = [this](udp_rx_msg_t msg){ proc_detector_type( msg ); };
    instr_map_[RD_MARS_RDOUT_ENB]  = [this](udp_rx_msg_t msg){ proc_mars_rdout_enb( msg ); };
    instr_map_[WR_MARS_RDOUT_ENB]  = [this](udp_rx_msg_t msg){ proc_mars_rdout_enb( msg ); };
    instr_map_[RD_TRIG]            = [this](udp_rx_msg_t msg){ proc_trig( msg ); };
    instr_map_[WR_TRIG]            = [this](udp_rx_msg_t msg){ proc_trig( msg ); };
    instr_map_[RD_FRAME_NO]        = [this](udp_rx_msg_t msg){ proc_frame_no( msg ); };
    instr_map_[WR_FRAME_NO]        = [this](udp_rx_msg_t msg){ proc_frame_no( msg ); };
    instr_map_[RD_COUNT_TIME_LO]   = [this](udp_rx_msg_t msg){ proc_count_time_lo( msg ); };
    instr_map_[WR_COUNT_TIME_LO]   = [this](udp_rx_msg_t msg){ proc_count_time_lo( msg ); };
    instr_map_[RD_COUNT_TIME_HI]   = [this](udp_rx_msg_t msg){ proc_count_time_hi( msg ); };
    instr_map_[WR_COUNT_TIME_HI]   = [this](udp_rx_msg_t msg){ proc_count_time_hi( msg ); };
    instr_map_[WR_MARS_CONF_LOAD]  = [this](udp_rx_msg_t msg){ proc_mars_conf_load( msg ); };
    instr_map_[WR_ADC_SPI]         = [this](udp_rx_msg_t msg){ proc_adc_spi( msg ); };
    instr_map_[RD_VERSIONREG]      = [this](udp_rx_msg_t msg){ proc_versionreg( msg ); };
    instr_map_[RD_MARS_PIPE_DELAY] = [this](udp_rx_msg_t msg){ proc_mars_pipe_delay( msg ); };
    instr_map_[WR_MARS_PIPE_DELAY] = [this](udp_rx_msg_t msg){ proc_mars_pipe_delay( msg ); };
    instr_map_[RD_TD_CAL]          = [this](udp_rx_msg_t msg){ proc_td_cal( msg ); };
    instr_map_[WR_TD_CAL]          = [this](udp_rx_msg_t msg){ proc_td_cal( msg ); };
    instr_map_[RD_COUNT_MODE]      = [this](udp_rx_msg_t msg){ proc_count_mode( msg ); };
    instr_map_[WR_COUNT_MODE]      = [this](udp_rx_msg_t msg){ proc_count_mode( msg ); };
    instr_map_[RD_CALPULSE_RATE]   = [this](udp_rx_msg_t msg){ proc_calpulse_rate( msg ); };
    instr_map_[WR_CALPULSE_RATE]   = [this](udp_rx_msg_t msg){ proc_calpulse_rate( msg ); };
    instr_map_[RD_CALPULSE_WIDTH]  = [this](udp_rx_msg_t msg){ proc_calpulse_width( msg ); };
    instr_map_[WR_CALPULSE_WIDTH]  = [this](udp_rx_msg_t msg){ proc_calpulse_width( msg ); };
    instr_map_[RD_CALPULSE_CNT]    = [this](udp_rx_msg_t msg){ proc_calpulse_cnt( msg ); };
    instr_map_[WR_CALPULSE_CNT]    = [this](udp_rx_msg_t msg){ proc_calpulse_cnt( msg ); };
    instr_map_[RD_MARS_CALPULSE]   = [this](udp_rx_msg_t msg){ proc_mars_calpulse( msg ); };
    instr_map_[WR_MARS_CALPULSE]   = [this](udp_rx_msg_t msg){ proc_mars_calpulse( msg ); };
    instr_map_[RD_CALPULSE_MODE]   = [this](udp_rx_msg_t msg){ proc_calpulse_mode( msg ); };
    instr_map_[WR_CALPULSE_MODE]   = [this](udp_rx_msg_t msg){ proc_calpulse_mode( msg ); };
    instr_map_[RD_UDP_IP_ADDR]     = [this](udp_rx_msg_t msg){ proc_udp_ip_addr( msg ); };
    instr_map_[WR_UDP_IP_ADDR]     = [this](udp_rx_msg_t msg){ proc_udp_ip_addr( msg ); };
    instr_map_[RD_EVENT_FIFO_CNT]  = [this](udp_rx_msg_t msg){ proc_event_fifo_cnt( msg ); };
    instr_map_[RD_EVENT_FIFO_DATA] = [this](udp_rx_msg_t msg){ proc_event_fifo_data( msg ); };

    // Create interfaces

    ps_i2c0_ = std::make_shared<PSI2C>("I2C0");
    ps_i2c1_ = std::make_shared<PSI2C>("I2C1");
}



//===============================================================
//  EVENT_FIFO_CTRL
//===============================================================
void proc_event_fifo_ctrl( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  DETECTOR_TYPE
//===============================================================
void proc_detector_type( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  MARS_RDOUT_ENB
//===============================================================
void proc_mars_rdout_enb( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  TRIG
//===============================================================
void proc_trig( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  FRAME_NO
//===============================================================
void proc_frame_no( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  COUNT_TIME_LO
//===============================================================
void proc_count_time_lo( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  COUNT_TIME_HI
//===============================================================
void proc_count_time_hi( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  MARS_CONF_LOAD
//===============================================================
void proc_mars_conf_load( const udp_rx_msg_t& msg )
{
    ( msg );
}
//===============================================================

//===============================================================
//  ADC_SPI
//===============================================================
void proc_adc_spi( const udp_rx_msg_t& msg )
{
    ( msg );
}
//===============================================================

//===============================================================
//  VERSIONREG
//===============================================================
void proc_versionreg( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  MARS_PIPE_DELAY
//===============================================================
void proc_mars_pipe_delay( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  TD_CAL
//===============================================================
void proc_td_cal( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  COUNT_MODE
//===============================================================
void proc_count_mode( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  CALPULSE_RATE
//===============================================================
void proc_calpuse_rate( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  CALPULSE_WIDTH
//===============================================================
void proc_calpulse_width( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  CALPULSE_CNT
//===============================================================
void proc_calpulse_cnt( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  MARS_CALPULSE
//===============================================================
void proc_mars_calpulse( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  CALPULSE_MODE
//===============================================================
void proc_calpulse_mode( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  UDP_IP_ADDR
//===============================================================
void proc_udp_ip_addr( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================

//===============================================================
//  EVENT_FIFO_CNT
//===============================================================
void proc_event_fifo_cnt( const udp_rx_msg_t& msg )
{
    single_reg_acc_req_proc( msg );
}
//===============================================================


//===============================================================
// Only UDP tasks for GeDetector.
//===============================================================
void GeDetector::task_init()
{
	xTaskCreate( udp_rx_task,
                 ( const char * ) "UDP_RX",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY,
				 &udp_rx_task_handle_ );

	xTaskCreate( udp_tx_task,
				 ( const char * ) "UDP_TX",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &udp_tx_task_handle_ );

    xTaskCreate( psi2c_task,
                 ( const char* ) "PS_I2C0"),
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 &psi2c_0_task_handler_ );

    xTaskCreate( psi2c_task,
                ( const char* ) "PS_I2C1"),
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                &psi2c_1_task_handler_ );


    xTaskCreate( psxadc_task,
                ( const char* ) "PS_I2C0"),
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                &psxadc_task_handler_ );
}
//===============================================================


void 

void GeDetector::rx_msg_proc( udp_rx_msg_t& udp_msg )
{
    op = udp_msg.op >> 14;
    reg = udp_msg.op && 0x3F;

    fast_access_resp_t fast_access_resq;

    int instr = udp_msg.op;
    auto it = instr_map_.find( instr );
    if( it != instr_map_.end() )
    {
        it->second( udp_msg );
    }
    else
    {
        std::cout << "Invalid instruction: " << instr << '\n';
    }
}

static void udp_tx_task_wrapper(void* param)
{
    auto obj = static_cast<Germanium*>(param);  // get `this` of Germanium
    obj->udp_tx_task();
}

static void Germanium::udp_tx_task()
{

}

void GeDetector::tx_msg_proc( )
{
    xQueueReceive( register_single_access_resp_queue,
    			   Recdstring,
                   portMAX_DELAY );
}
