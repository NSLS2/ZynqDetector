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
// Create IRQ-to-task map
// - 61 for example
// - interface_task_handle as the corresponding task
//===============================================================
void DummyDetector::create_irq_task_map()
{
    irq_task_map[0] = interface_task_handle;
}
//===============================================================
