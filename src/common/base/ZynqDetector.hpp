#pragma once

// C++ includes.
#include <vector>
#include <cstdint>
#include <memory>
// FreeRTOS includes.
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "FreeRTOS_IP.h"
#include "network_interface.h"
#include "ff.h"  // FatFS file system library for SD card
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"

#include "msg.hpp"
#include "fpga.hpp"
#include "network.hpp"

class Udp_Msg_Handler
{
private:

public:
    Udp_Msg_Handler();

};


template< typename Derived >
class ZynqDetector
{
protected:

    //==================================================
    //                    Variables                   //
    //==================================================

    //------------------------------
    // Queues
    //------------------------------
    const size_t REGISTER_SINGLE_ACCESS_REQ_QUEUE_LENG = 100;
    const size_t REGISTER_SINGLE_ACCESS_REQ_QUEUE_SIZE = SINGLE_REGISTER_ACCESS_REQ_QUEUE_LENG
                                                       * sizeof( SingleRegisterAccessReq );

    const size_t REGISTER_SINGLE_ACCESS_RESP_QUEUE_LENG = 100;
    const size_t REGISTER_SINGLE_ACCESS_RESP_QUEUE_SIZE = SINGLE_REGISTER_ACCESS_RESP_QUEUE_LENG * sizeof( SingleRegisterAccessResp );

    /*
    const size_t PL_INTERFACE_SINGLE_ACCESS_REQ_QUEUE_LENG = 10;
    const size_t PL_INTERFACE_SINGLE_ACCESS_REQ_QUEUE_SIZE = PL_INTERFACE_SINGLE_ACCESS_REQ_QUEUE_LENG
                                                           * sizeof( PlInterfaceSingleAccessReq );

    const size_t PL_INTERFACE_SINGLE_ACCESS_RESP_QUEUE_LENG = 10;
    const size_t PL_INTERFACE_SINGLE_ACCESS_RESP_QUEUE_SIZE = PL_INTERFACE_SINGLE_ACCESS_RESP_QUEUE_LENG
                                                            * sizeof( PlInterfaceMultiAccessResp );

    const size_t PL_INTERFACE_MULTI_ACCESS_REQ_QUEUE_LENG = 5;
    const size_t PL_INTERFACE_MULTI_ACCESS_REQ_QUEUE_SIZE = PL_INTERFACE_MULTI_ACCESS_REQ_QUEUE_LENG
                                                          * sizeof( PlInterfaceSingleAccessReq );

    const size_t PL_INTERFACE_MULTI_ACCESS_RESP_QUEUE_LENG = 5;
    const size_t PL_INTERFACE_MULTI_ACCESS_RESP_QUEUE_SIZE = PL_INTERFACE_MULTI_ACCESS_RESP_QUEUE_LENG
                                                           * sizeof( PlInterfaceMultiAccessResp );
    */
    // Queue handlers
    QueueHandle_t register_single_access_req_queue_  = NULL;
    QueueHandle_t register_single_access_resp_queue_ = NULL;

    QueueSetMemberHandle_t active_resp_queue_;
    QueueSetHandle_t       resp_queue_set_;
        
    
    //------------------------------
    // Task handlers
    //------------------------------
    TaskHandle_t  udp_rx_task_handle_;
    TaskHandle_t  udp_tx_task_handle_;
    TaskHandle_t  register_single_access_task_handle_;
    TaskHandle_t  slow_access_task_handle_;

    
    //------------------------------
    // Network
    //------------------------------
    std::unique_ptr<Network> network_;
    std::unique_ptr<Zynq>    zynq_;

    TimerHandle_t xPollTimer_ = NULL;
    std::vector<uint16_t> poll_list_{};  // PVs to be polled

    Logger logger;

    //==================================================
    //                    Functions                   //
    //==================================================
    //------------------------------
    // Interrupt
    //------------------------------
    virtual static void ISR_wrapper(void* context) = 0;


    //------------------------------
    // Interrupt
    //------------------------------
    virtual void initialize_instr_map() = 0;
    virtual void isr_handler() = 0;


    //------------------------------
    // Task
    //------------------------------
    void network_task_init();
    virtual void create_detector_queues() = 0;
    virtual void create_device_access_tasks() = 0;
    virtual void polling_task_init() = 0;

    virtual void register_single_access_task();
    //virtual void pl_if_single_access_task( void *pvParameters ) = 0;
    //virtual void pl_if_multi_access_task()( void *pvParameters ) = 0;

    //------------------------------
    // Message
    //------------------------------
    //void rx_msg_proc( std::any& msg );
    //void single_reg_acc_req_proc( udp_rx_msg_t& msg )

    //------------------------------
    // General
    //------------------------------
    void must_override();

    // Write status code to register.
    void set_status( uint32_t status );

    template <typename T>
    void report_error( const std::string& s, T err_code, uint32_t fail_num );
    //==================================================
    

public:

    ZynqDetector( uint32_t base_addr );
    ~ZynqDetector();

    void DummyDetector::create_irq_task_map();
    
    void network_init();
    virtual void queue_init() = 0;
    virtual void interrupt_init() = 0;
    virtual void task_init() = 0;

    //===============================================================
    // Used by both ZynqDetector or derived detector classes to wrap
    // a task function task(), so that the task can access the
    // members in ZynqDetector or in a derived detector class:
    // - If task() is defined in ZynqDetector only,
    //   ZynqDetector::task() is called;
    // - If task() is defined in ZynqDetector but overridden in
    //   Derived, Derived::task() is called;
    //   ZynqDetector::task() is called;
    // - If task() is defined in Derived only,
    //   Derived::task() is called.
    //
    // This wrapper can be used by both ZynqDetector or derived
    // detectors.
    //
    // Parameters:
    //   - param: `this` of the caller.
    //   - task: pointer to the task function.
    //===============================================================
    //static void ZynqDetector::task_wrapper(void* param, void (Derived::*task)());
};

