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
#include "ZynqDetector.hpp"
#include "Germanium.hpp"
#include "pynq_ssd_msg.hpp"

#define TIMER_ID 1
#define DELAY_10_SECONDS 10000UL
#define DELAY_1_SECOND 1000UL
#define TIMER_CHECK_THRESHOLD 9
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
#define QUEUE_BUFFER_SIZE 100

uint8_t ucQueueStorageArea[QUEUE_BUFFER_SIZE];
StackType_t xStack1[configMINIMAL_STACK_SIZE];
StackType_t xStack2[configMINIMAL_STACK_SIZE];
StaticTask_t xTxBuffer, xRxBuffer;
StaticTimer_t xTimerBuffer;
static StaticQueue_t xStaticQueue;
#endif

Germanium::Germanium()
    : Base(std::make_unique<GermaniumNetwork>()), zynq_(base_addr_), i2c0_(zynq.add_ps_i2c(0, psi2c0_req_queue, psi2c0_resp_queue)), i2c1_(zynq.add_ps_i2c(1, psi2c1_req_queue, psi2c1_resp_queue)), ltc2309(psi2c_1, LTC2309_I2C_ADDR, true, psi2c_1_req_queue, chan_assign), dac7678(psi2c_1, DAC7678_I2C_ADDR, psi2c_1_req_queue, chan_assign), tmp100_0_(psi2c_0, TMP100_0_I2C_ADDR, psi2c_0_req_queue), tmp100_1_(psi2c_0, TMP100_1_I2C_ADDR, psi2c_0_req_queue), tmp100_2_(psi2c_0, TMP100_2_I2C_ADDR, psi2c_0_req_queue)
{
    instr_map_[RD_EVENT_FIFO_CTRL] = [this](UDPRxMsg msg) { proc_event_fifo_ctrl(msg); };
    instr_map_[WR_EVENT_FIFO_CTRL] = [this](UDPRxMsg msg) { proc_event_fifo_ctrl(msg); };
    instr_map_[RD_DETECTOR_TYPE]   = [this](UDPRxMsg msg) { proc_detector_type(msg); };
    instr_map_[WR_DETECTOR_TYPE]   = [this](UDPRxMsg msg) { proc_detector_type(msg); };
    instr_map_[RD_MARS_RDOUT_ENB]  = [this](UDPRxMsg msg) { proc_mars_rdout_enb(msg); };
    instr_map_[WR_MARS_RDOUT_ENB]  = [this](UDPRxMsg msg) { proc_mars_rdout_enb(msg); };
    instr_map_[RD_TRIG]            = [this](UDPRxMsg msg) { proc_trig(msg); };
    instr_map_[WR_TRIG]            = [this](UDPRxMsg msg) { proc_trig(msg); };
    instr_map_[RD_FRAME_NO]        = [this](UDPRxMsg msg) { proc_frame_no(msg); };
    instr_map_[WR_FRAME_NO]        = [this](UDPRxMsg msg) { proc_frame_no(msg); };
    instr_map_[RD_COUNT_TIME_LO]   = [this](UDPRxMsg msg) { proc_count_time_lo(msg); };
    instr_map_[WR_COUNT_TIME_LO]   = [this](UDPRxMsg msg) { proc_count_time_lo(msg); };
    instr_map_[RD_COUNT_TIME_HI]   = [this](UDPRxMsg msg) { proc_count_time_hi(msg); };
    instr_map_[WR_COUNT_TIME_HI]   = [this](UDPRxMsg msg) { proc_count_time_hi(msg); };
    instr_map_[WR_MARS_CONF_LOAD]  = [this](UDPRxMsg msg) { proc_mars_conf_load(msg); };
    instr_map_[WR_ADC_SPI]         = [this](UDPRxMsg msg) { proc_adc_spi(msg); };
    instr_map_[RD_VERSIONREG]      = [this](UDPRxMsg msg) { proc_versionreg(msg); };
    instr_map_[RD_MARS_PIPE_DELAY] = [this](UDPRxMsg msg) { proc_mars_pipe_delay(msg); };
    instr_map_[WR_MARS_PIPE_DELAY] = [this](UDPRxMsg msg) { proc_mars_pipe_delay(msg); };
    instr_map_[RD_TD_CAL]          = [this](UDPRxMsg msg) { proc_td_cal(msg); };
    instr_map_[WR_TD_CAL]          = [this](UDPRxMsg msg) { proc_td_cal(msg); };
    instr_map_[RD_COUNT_MODE]      = [this](UDPRxMsg msg) { proc_count_mode(msg); };
    instr_map_[WR_COUNT_MODE]      = [this](UDPRxMsg msg) { proc_count_mode(msg); };
    instr_map_[RD_CALPULSE_RATE]   = [this](UDPRxMsg msg) { proc_calpulse_rate(msg); };
    instr_map_[WR_CALPULSE_RATE]   = [this](UDPRxMsg msg) { proc_calpulse_rate(msg); };
    instr_map_[RD_CALPULSE_WIDTH]  = [this](UDPRxMsg msg) { proc_calpulse_width(msg); };
    instr_map_[WR_CALPULSE_WIDTH]  = [this](UDPRxMsg msg) { proc_calpulse_width(msg); };
    instr_map_[RD_CALPULSE_CNT]    = [this](UDPRxMsg msg) { proc_calpulse_cnt(msg); };
    instr_map_[WR_CALPULSE_CNT]    = [this](UDPRxMsg msg) { proc_calpulse_cnt(msg); };
    instr_map_[RD_MARS_CALPULSE]   = [this](UDPRxMsg msg) { proc_mars_calpulse(msg); };
    instr_map_[WR_MARS_CALPULSE]   = [this](UDPRxMsg msg) { proc_mars_calpulse(msg); };
    instr_map_[RD_CALPULSE_MODE]   = [this](UDPRxMsg msg) { proc_calpulse_mode(msg); };
    instr_map_[WR_CALPULSE_MODE]   = [this](UDPRxMsg msg) { proc_calpulse_mode(msg); };
    instr_map_[RD_UDP_IP_ADDR]     = [this](UDPRxMsg msg) { proc_udp_ip_addr(msg); };
    instr_map_[WR_UDP_IP_ADDR]     = [this](UDPRxMsg msg) { proc_udp_ip_addr(msg); };
    instr_map_[RD_EVENT_FIFO_CNT]  = [this](UDPRxMsg msg) { proc_event_fifo_cnt(msg); };
    instr_map_[RD_EVENT_FIFO_DATA] = [this](UDPRxMsg msg) { proc_event_fifo_data(msg); };
    isntr_map_[WR_GLOBALSTR]       = [this](UDPRxMsg msg){ proc_globalstr_update(msg); }
    isntr_map_[WR_CHANNELSTR]      = [this](UDPRxMsg msg){ proc_channelstr_update(msg); }

    // Create interfaces

    network_init(std::make_unique<GermaniumNetwork>(this));

    ps_i2c0_ = std::make_shared<PSI2C>("I2C0");
    ps_i2c1_ = std::make_shared<PSI2C>("I2C1");
}

//===============================================================
//  EVENT_FIFO_CTRL
//===============================================================
void proc_event_fifo_ctrl(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  DETECTOR_TYPE
//===============================================================
void proc_detector_type(const UDPRxMsg& msg)
{
    if ( msg.op&0x8000 == 0 )
    {
        if ( msg.reg_acc_reg_data == 0 )
        {
            nelm_ = 192;
            num_chips_ = 6;
        }
        else
        {
            nelm_ = 384;
            num_chips_ = 12;
        }
    }
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  MARS_RDOUT_ENB
//===============================================================
void proc_mars_rdout_enb(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  TRIG
//===============================================================
void proc_trig(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  FRAME_NO
//===============================================================
void proc_frame_no(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  COUNT_TIME_LO
//===============================================================
void proc_count_time_lo(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  COUNT_TIME_HI
//===============================================================
void proc_count_time_hi(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  MARS_CONF_LOAD
//===============================================================
void proc_mars_conf_load(const UDPRxMsg& msg)
{
    (msg);
}
//===============================================================

//===============================================================
//  ADC_SPI
//===============================================================
void proc_adc_spi(const UDPRxMsg& msg)
{
    (msg);
}
//===============================================================

//===============================================================
//  VERSIONREG
//===============================================================
void proc_versionreg(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  MARS_PIPE_DELAY
//===============================================================
void proc_mars_pipe_delay(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  TD_CAL
//===============================================================
void proc_td_cal(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  COUNT_MODE
//===============================================================
void proc_count_mode(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  CALPULSE_RATE
//===============================================================
void proc_calpuse_rate(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  CALPULSE_WIDTH
//===============================================================
void proc_calpulse_width(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  CALPULSE_CNT
//===============================================================
void proc_calpulse_cnt(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  MARS_CALPULSE
//===============================================================
void proc_mars_calpulse(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  CALPULSE_MODE
//===============================================================
void proc_calpulse_mode(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  UDP_IP_ADDR
//===============================================================
void proc_udp_ip_addr(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================

//===============================================================
//  EVENT_FIFO_CNT
//===============================================================
void proc_event_fifo_cnt(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================


//===============================================================
//  EVENT_FIFO_CNT
//===============================================================
void proc_event_fifo_cnt(const UDPRxMsg& msg)
{
    register_access_request_proc(msg);
}
//===============================================================


//===============================================================
//===============================================================
void proc_channelstr_update( const UDPRxMsg& msg )
{
    memcpy( &channelstr__, msg.payload.channelstr_, sizeof(channelstr__) );
}
//===============================================================


//===============================================================
//===============================================================
void proc_globalstr_update( const UDPRxMsg& msg )
{
    memcpy( &globalstr__, msg.payload.lglobalstr__, sizeof(globalstr__) );
}
//===============================================================


//===============================================================
//===============================================================
void register_access_request_proc(const UDPRxMsg &msg)
{
    RegisterAccessRequest req;
    req.op = msg.op;
    xQueueSend(register_access_request_queue, req,
               , 0UL)
}
//===============================================================


//===============================================================
// Only UDP tasks for GeDetector.
//===============================================================
void GeDetector::task_init()
{
    xTaskCreate(udp_rx_task,
                (const char *)"UDP_RX",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                &udp_rx_task_handle_);

    xTaskCreate(udp_tx_task,
                (const char *)"UDP_TX",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                &udp_tx_task_handle_);

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

void Germanium::create_device_access_tasks()
{
    reg_.create_register_single_access_task();

    psxadc_.create_psxadc_task();

    psi2c0_.create_psi2c_task();
    psi2c1_.create_psi2c_task();

    xTaskCreate(register_multi_access_task_wrapper, (const char *)"Register-Multi-Access", configMINIMAL_STACK_SIZE, this, tskIDLE_PRIORITY, &register_multi_access_task_handle_);

    xTaskCreate(psxadc_.task_wrapper, (const char *)"PSXADC-Access", configMINIMAL_STACK_SIZE, &psxadc_, tskIDLE_PRIORITY, &psxadc_task_handle_);

    xTaskCreate(psi2c0_.task_wrapper, (const char *)"PSI2C-0-Access", configMINIMAL_STACK_SIZE, &psi2c0_, tskIDLE_PRIORITY, &psi2c_0_task_handle_);

    xTaskCreate(psi2c1_.task_wrapper, (const char *)"PSI2C-1-Access", configMINIMAL_STACK_SIZE, &psi2c1_, tskIDLE_PRIORITY, &psi2c_1_task_handle_);
}
//===============================================================


//===============================================================
// Register multi-access task wrapper
//===============================================================
void Register::create_register_multi_access_task()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { register_multi_access_task(); });
    xTaskCreate( task_wrapper, "Register Single Access", 1000, &task_func, 1, NULL );
}
//===============================================================


//===============================================================
// Register multi-access task
//===============================================================
void Germanium::register_multi_access_task()
{
    GermaniumAccessReq req;

    xQueueReceive( reg_multi_access_resp_queue_, &req, 0);
    switch ( req.op & 0x7fff )
    {
        case UPDATE_LOADS:
            update_loads( &req.data )
            break;

        case STUFF_MARS:
            stuff_mars();
            break;

        case AD9252_CNFG:
            ad9252_cnfg( req.ad9252_cnfg.chip_num
                       , req.ad9252_cnfg.addr
                       , req.ad9252_cnfg.val );

        case ZDDM_ARM:
            zddm_arm( req.zddm_arm.mode
                    , req.zddm_arm.val );
            break;
            
        default:
            log_error( "Invalid op: %d", req.op & 0x7fff );
    }
}
//===============================================================


//===============================================================
// Germanium queue init
//===============================================================
void Germanium::create_detector_queues()
{
    psi2c_0_req_queue = xQueueCreate(5, sizeof(PSI2CReq));
    psi2c_1_req_queue = xQueueCreate(5, sizeof(PSI2CReq));
    psxadc_req_queue  = xQueueCreate(5, sizeof(PSXADCReq));

    psi2c_0_resp_queue = xQueueCreate(5, sizeof(PSI2CResp));
    psi2c_1_resp_queue = xQueueCreate(5, sizeof(PSI2CResp));
    psxadc_resp_queue  = xQueueCreate(5, sizeof(PSXADCResp));

    resp_queue_set = xQueueCreateSet(50);

    xQueueAddToSet(psi2c_0_resp_queue, resp_queue_set);
    xQueueAddToSet(psi2c_1_resp_queue, resp_queue_set);
    xQueueAddToSet(psxadc_resp_queue, resp_queue_set);
}
//===============================================================


void polling_task_init()
{
    poll_list.emplace_back( HV_RBV | 0x8000 );
    poll_list.emplace_back( HV_CURR | 0x8000  );

    TimerHandle_t xTimer = xTimerCreate( "1S Polling Tmer"
        , pdMS_TO_TICKS(1000) // 1 second
        , pdTRUE              // auto-reload
        , ( void * ) 1        // timer ID
        , polling_1s );
}

void polling_1s()
{
    UDPRxMsg msg;
    
    for ( auto iter : poll_list )
    {
        int instr = *iter;
        auto it = instr_map_.find(instr);
        if (it != instr_map_.end())
        {
            msg.op = instr;
            it->second(msg);  // Call the corresponding function
        }
        else
        {
            std::cout << "Unknown instruction: " << instr << '\n';
        }
    }
}

//===============================================================
// Germanium operations
//===============================================================
void Germanium::latch_conf()
{
    reg_.write( MARS_CONF_LOAD, 2 );
    reg_.write( MARS_CONF_LOAD, 0 );
}

void Germanium::update_loads( char* loads )
{
    memcpy( loads_, loads, sizeof(loads_) );
}


void Germanium::stuff_mars()
{
    for (int i = 0; i < 12; i++)
    {
        reg_.write( MARS_CONF_LOAD, 4 );
        reg_.write( MARS_CONF_LOAD, 0 );

        for (int j = 0; j < 14; j++)
        {
            reg_.write( MARS_CONFIG, loads__[i][j] );
            latch_conf();
            usleep(1000);
        }

        reg_.write( MARS_CONF_LOAD, 0x00010000 << i );
        reg_.write( MARS_CONF_LOAD, 0 );
        usleep( 1000 );
    }
}

void Germanium::send_spi_bit( int chip_sel, int val )
{
    sda = val & 0x1;

    // set sclk low
    reg_.write(ADC_SPI, (chip_sel | 0));

    // set data with clock low
    reg_.write(ADC_SPI, (chip_sel | sda));

    // set clk high
    reg_.write(ADC_SPI, (chip_sel | 0x2 | sda));

    // set clk low
    reg_.write(ADC_SPI, (chip_sel | sda));

    // set data low
    reg_.write(ADC_SPI, (chip_sel | 0));
}

void Germanium::load_ad9252reg( int chip_sel, int addr, int data )
{
    int i, j, k;

    // small delay
    for (k = 0; k < 100; k++)
        ;

    // Read/Write bit
    send_spi_bit(chip_sel, 0);

    // W1=W0=0 (word length = 1 byte)
    for (i = 1; i >= 0; i--)
        send_spi_bit(chip_sel, 0);

    // address
    for (j = 12; j >= 0; j--)
        send_spi_bit(chip_sel, addr >> j);

    // data
    for (j = 7; j >= 0; j--)
        send_spi_bit(chip_sel, data >> j);

    // small delay
    for (k = 0; k < 100; k++)
        ;
    return (0);
}

int Germanium::ad9252_cnfg( int chip_num, int addr, int data )
{

    int chip_sel;
    extern int fd;

    // chip_sel is defined as: bit2=ADC0, bit3=ADC1, bit4=ADC2
    /*
    if ( chip_num>0 && chip_num<4 )
    {
        chip_sel = (7-chip_num) << 2;
    }
    else
    {
        chip_sel = 0x0;
    }
    */
    if (chip_num == 1)
        chip_sel = 0b11000;
    else if (chip_num == 2)
        chip_sel = 0b10100;
    else if (chip_num == 3)
        chip_sel = 0b01100;
    else
        chip_sel = 0b00000;

    // Assert CSB
    reg_.write( ADC_SPI, chip_sel );
    load_ad9252reg( chip_sel, addr, data) ;
    reg_.write( ADC_SPI, 0b11100 );

    return (0);
}

void Germanium::zddm_arm( int mode, int val )
{
    // Request to read FRAME_NO

    // Request to write TRIG
}
