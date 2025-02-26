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
void ZynqDetector::interrupt_init()
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
// Message queue initialization.
//===============================================================
void DummyDetector::queue_init()
{
    //==================================================
    // This function definition is for reference only,
    // and must be overriden by the derived class.

    //==================================================

    // Create queues
	single_register_access_req_queue      = xQueueCreate( 100, sizeof( SingleRegisterAccessReq ) );
	pl_interface_single_access_req_queue  = xQueueCreate( 100, sizeof( PlInterfaceSingleAccessReq ) );
    pl_interface_multi_access_req_queue   = xQueueCreate( 4,   sizeof( PlInterfaceMultiAccessReq ) );
    ps_interface_access_req_queue         = xQueueCreate( 4,   sizeof( PsInterfaceAccessReq ) );
	single_register_access_resp_queue     = xQueueCreate( 100, sizeof( SingleRegisterAccessResp ) );
	pl_interface_single_access_resp_queue = xQueueCreate( 100, sizeof( PlInterfaceSingleAccessResp ) );
    pl_interface_multi_access_resp_queue  = xQueueCreate( 4,   sizeof( PlInterfaceMultiAccessResp ) );
    ps_interface_access_req_queue         = xQueueCreate( 4,   sizeof( PsInterfaceAccessResp ) );

    // Create queue sets
    resp_queue_set = xQueueCreateSet( 
        REG_ACCESS_RESP_QUEUE_SIZE +
        INTERFACE_SINGLE_ACCESS_RESP_QUEUE_SIZE +
        INTERFACE_MULTI_ACCESS_RESP_QUEUE_SIZE );

    xQueueAddToSet( single_register_access_resp_queue, resp_queue_set );
    xQueueAddToSet( pl_interface_single_access_resp_queue, resp_queue_set );
    xQueueAddToSet( pl_interface_multi_access_resp_queue, resp_queue_set );
    xQueueAddToSet( ps_interface_access_req_queue, resp_queue_set );
}
//===============================================================


//===============================================================
//===============================================================
void ZynqDetector:task_init()
{
    //==================================================
    // This function definition is for reference only.
    // A derived class must override.
    throw std::runtime_error( __PRETTY_FUNCTION__
        + " for reference only. Implement it for the derived class." );
    //==================================================

	xTaskCreate( udp_rx_task, 				 // The function that implements the task.
                 ( const char * ) "UDP_RX",  // Text name for the task, provided to assist debugging only.
				 configMINIMAL_STACK_SIZE,   // The stack allocated to the task.
				 NULL, 					     // The task parameter is not used, so set to NULL.
				 tskIDLE_PRIORITY,			 // The task runs at the idle priority.
				 &udp_rx_task_handle );

	xTaskCreate( udp_tx_task,
				 ( const char * ) "UDP_TX",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &udp_tx_task_handle );

	xTaskCreate( single_register_access_task,
				 ( const char * ) "SINGLE_REGISTER_ACCESS",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &reg_access_task_handle );

    xTaskCreate( pl_if_signle_access_task,
                 ( const char * ) "PL_IF_SINGLE_ACCESS",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 &interface_single_access_task_handle );

	xTaskCreate( pl_if_multi_access_task,
				 ( const char * ) "PL_IF_MULTI_ACCESS",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &interface_multi_access_task_handle );
}
//===============================================================


//===============================================================
// This task performs single register read/write operation.
//===============================================================
void ZynqDetector::pl_if_single_access_task( void *pvParameters )
{
    interface_single_access_req_t  req;
    interface_single_access_resp_t resp;

    auto param = static_cast<interface_single_access_task_param*>(pvParameters);

    while(1)
    {
        xQueueReceive( 	static_cast<QueueHandle_t*>(param).req_queue,
						&req,
						portMAX_DELAY );
        
        if ( req.read )
        {
            interface_read( req.device_addr, req.data );
            resp.op = req.op;

            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            resp.data = reg.rd( req.addr );

            xQueueSend( static_cast<QueueHandle_t*>(param).resp_queue,
                        static_cast<const void*>(resp),
                        portMAX_DELAY );
        }
    }
}
//===============================================================

//===============================================================
// This task processes requests that need to continuously 
// access a device multiple times, e.g., to configure a device
// through an I2C bus while the configuration data consists of
// multiple dwords.
//===============================================================
void ZynqDetector::pl_if_multi_access_task()( void *pvParameters )
{
    interface_multi_access_req_t  req;
    interface_multi_access_resp_t resp;

    while(1)
    {
        xQueueReceive( 	static_cast<QueueHandle_t*>(param).req_queue,
						&req,
                        portMAX_DELAY );

        // compose the instruction queue
        
        for ( int i=0; !instr_queue.empty(); ++i )
        {
            auto instr = instr_queue.front();
            instr_queue.pop();
            if ( instr.read )
            {
                read( instr.device_addr, instr.reg_addr );
            }
            else
            {
                write( instr.device_addr, instr.reg_addr, instr.data );
            }

            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if ( instr.read )
            {
                resp.data[i] = reg.rd( req.addr );
            }
        }

        if( req.read )
        {
            resp.op = req.op;
            
            xQueueSend( static_cast<QueueHandle_t*>(param).resp_queue,
                        static_cast<const void*>(resp),
                        portMAX_DELAY );
        }
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






//===============================================================
// UDP transmit task.
//===============================================================
void DummyDetector::udp_tx_task( void *pvParameters )
{
    //==================================================
    // This function definition is for reference only.
    // A derived class must override.
    throw std::runtime_error( __PRETTY_FUNCTION__
        + " for reference only. Implement it for the derived class." );
    //==================================================
    
    struct freertos_sockaddr dest_sock_addr;
    socklen_t dest_sock_addr_leng = sizeof(xDestination);
    int32_t bytesSent;

    dest_sock_addr.sin_addr = FreeRTOS_inet_addr_quick( svr_ip_addr[3],
                                                        svr_ip_addr[2],
                                                        svr_ip_addr[1],
                                                        svr_ip_addr[0] );
    dest_sock_addr.sin_port = FreeRTOS_htons(UDP_PORT);

    udp_tx_msg_t msg;
    
    while(1)
    {
        tx_msg_proc( msg );
        tx_leng = FreeRTOS_sendto( udp_socket,
                                   msg,
                                   msg.leng + 4, // ID (2) + OP (2) + data
                                   0,
                                   &dest_sock_addr,
                                   dest_sock_addr_leng );
        if ( tx_leng <= 0 )
        {
            std::err << "Failed to send UDP message.\n";
        }
    }
}
//===============================================================


