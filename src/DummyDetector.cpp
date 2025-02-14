#include "DummyDetector.hpp"
#include <xscugic.h>  // Zynq GIC driver


extern TaskHandle_t irq_task_handles[];  // External map of task handles

//===============================================================
// Static function for handling ISR calls
//===============================================================
void DummyDetector::isr_wrapper(void* context)
{
    static_cast<DummyDetector*>(context)->isr_handler();
}
//===============================================================

//===============================================================
// ISR Handler - Processes interrupts
//===============================================================
void DummyDetector::isr_handler()
{
    // Get the interrupt ID from the GIC
    int irq_num = XScuGic_GetIntrID( &gic );
    irq_num -= 61

    if ( irq_task_map.count( irq_num ) )
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR( irq_task_handles[irq_num], &xHigherPriorityTaskWoken );
    }

    // Clear the interrupt in GIC
    XScuGic_ClearPending(&gic, irq_num + 61);

    // Context switch if necessary
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
//===============================================================

//===============================================================
// Register ISR with the interrupt controller
//===============================================================
void DummyDetector::register_isr()
{
    XScuGic_Config* gic_config;
    
    // Get GIC configuration from hardware
    gic_config = XScuGic_LookupConfig( XPAR_SCUGIC_0_DEVICE_ID );
    if ( gic_config == nullptr )
    {
        // Handle error
        return;
    }

    // Initialize GIC
    if ( XScuGic_CfgInitialize( &gic, gic_config, gic_config->CpuBaseAddress ) != XST_SUCCESS )
    {
        // Handle error
        return;
    }


    for (int irq = 61; irq <= MAX_PL_IRQ; irq++)
    {
        XScuGic_Connect(gic, irq, isr_wrapper, this);
        XScuGic_Enable(gic, irq);
    }
}
//===============================================================



//===============================================================
// UDP request handlers.
//===============================================================
void set_mars_conf_load()

//===============================================================
// Create IRQ-to-task map
// - 61 for example
// - interface_task_handle as the corresponding task
//===============================================================
void DummyDetector::create_irq_task_map()
{
    reg_req_map_[MARS_CONF_LOAD]   = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); mars_conf_load( op, value ); };
    reg_req_map_[LEDS]             = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); leds( op, value ); };
    reg_req_map_[MARS_CONFIG]      = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); mars_config( op, value ); };
    reg_req_map_[VERSIONREG]       = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); version( op, value ); };
    reg_req_map_[MARS_CALPULSE]    = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); mars_calpulse()( op, value ); };
    reg_req_map_[MARS_PIPE_DELAY]  = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); mars_pipe_delay()( op, value ); };
    reg_req_map_[MARS_RDOUT_ENB]   = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); mars_rdout_enb()( op, value ); };
    reg_req_map_[EVENT_TIME_CNTR]  = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); event_time_cntr()( op, value ); };
    reg_req_map_[SIM_EVT_SEL]      = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); sim_event( op, value ); };
    reg_req_map_[SIM_EVENT_RATE]   = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); sim_event_rate( op, value ); };
    reg_req_map_[ADC_SPI]          = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); adc_spi( op, value ); };
    reg_req_map_[CALPULSE_CNT]     = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); calpulse_cnt( op, value ); };
    reg_req_map_[CALPULSE_RATE]    = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); calpulse_rate( op, value ); };
    reg_req_map_[CALPULSE_WIDTH]   = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); calpulse_width( op, value ); };
    reg_req_map_[CALPULSE_MODE]    = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); calpulse_mode( op, value ); };
    reg_req_map_[TD_CAL]           = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); td_cal( op, value ); };
    reg_req_map_[UDP_IP_ADDR]      = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); udp_ip_addr( op, value ); };
    reg_req_map_[TRIG]             = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); trig( op, value ); };
    reg_req_map_[COUNT_TIME]       = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); count_time_lo( op, value ); };
    reg_req_map_[FRAME_NO]         = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); frame_no( op, value ); };
    reg_req_map_[COUNT_MODE]       = [this](std::any arg) { auto [op, value] = std::any_cast<int>(arg); count_mode( op, value ); };
    //req_map_[EVENT_FIFO_DATA]  = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[EVENT_FIFO_CNT]   = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[EVENT_FIFO_CNTRL] = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[DMA_CONTROL]      = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[DMA_STATUS]       = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[DMA_BASEADDR]     = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[DMA_BURSTLEN]     = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[DMA_BUFLEN]       = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[DMA_CURADDR]      = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[DMA_THROTTLE]     = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[DMA_IRQ_THROTTLE] = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[DMA_IRQ_ENABLE]   = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
    //req_map_[DMA_IRQ_COUNT]    = [this](std::any arg) { int value = std::any_cast<int>(arg); xxx;
}
//===============================================================


void DummyDetector::initialize_instr_map()
{

}
