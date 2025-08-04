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
//#include "FreeRTOS_IP.h"
//#include "network_interface.h"
//#include "ff.h"  // FatFS file system library for SD card
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"

#include "Logger.hpp"
#include "Register.hpp"
#include "Network.hpp"
#include "Zynq.hpp"



template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
class ZynqDetector
{
protected:

    //==================================================
    //                    Variables                   //
    //==================================================

    //------------------------------
    // Queues
    //------------------------------
    static constexpr size_t REGISTER_SINGLE_ACCESS_REQ_QUEUE_LENG = 100;
    static constexpr size_t REGISTER_SINGLE_ACCESS_REQ_QUEUE_SIZE = DerivedDetector::REGISTER_SINGLE_ACCESS_REQ_QUEUE_LENG
                                                                    * sizeof( DerivedDetector::RegisterSingleAccessReq );

    /*
    static constexpr size_t PL_INTERFACE_SINGLE_ACCESS_REQ_QUEUE_LENG = 10;
    static constexpr size_t PL_INTERFACE_SINGLE_ACCESS_REQ_QUEUE_SIZE = PL_INTERFACE_SINGLE_ACCESS_REQ_QUEUE_LENG
                                                                        * sizeof( PlInterfaceSingleAccessReq );

    static constexpr size_t PL_INTERFACE_SINGLE_ACCESS_RESP_QUEUE_LENG = 10;
    static constexpr size_t PL_INTERFACE_SINGLE_ACCESS_RESP_QUEUE_SIZE = PL_INTERFACE_SINGLE_ACCESS_RESP_QUEUE_LENG
                                                            * sizeof( PlInterfaceMultiAccessResp );

    static constexpr size_t PL_INTERFACE_MULTI_ACCESS_REQ_QUEUE_LENG = 5;
    static constexpr size_t PL_INTERFACE_MULTI_ACCESS_REQ_QUEUE_SIZE = PL_INTERFACE_MULTI_ACCESS_REQ_QUEUE_LENG
                                                                       * sizeof( PlInterfaceSingleAccessReq );

    static constexpr size_t PL_INTERFACE_MULTI_ACCESS_RESP_QUEUE_LENG = 5;
    static constexpr size_t PL_INTERFACE_MULTI_ACCESS_RESP_QUEUE_SIZE = PL_INTERFACE_MULTI_ACCESS_RESP_QUEUE_LENG
                                                                        * sizeof( PlInterfaceMultiAccessResp );
    */
    // Queue handlers
    QueueHandle_t  register_single_access_req_queue_  = NULL;
    QueueHandle_t  register_single_access_resp_queue_ = NULL;
        
    //------------------------------
    // Network
    //------------------------------
    std::unique_ptr<DerivedNetwork> network_;

    //------------------------------
    // Zynq
    //------------------------------
    std::unique_ptr<DerivedZynq>    zynq_;

    //TimerHandle_t xPollTimer_ = NULL;
    //std::vector<uint16_t> poll_list_{};  // PVs to be polled


    //==================================================
    //                    Functions                   //
    //==================================================
    //------------------------------
    // Interrupt
    //------------------------------
    //virtual static void ISR_wrapper(void* context) = 0;


    //------------------------------
    // Interrupt
    //------------------------------
    //virtual void initialize_instr_map() = 0;
    //virtual void isr_handler() = 0;


    //------------------------------
    // Task
    //------------------------------
    //void create_network_tasks();
    //void create_detector_queues();
    //void create_device_access_tasks();
    //void create_polling_tasks();

    //virtual void register_single_access_task();
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

    // Write status code to register.
    //void set_status( uint32_t status );

    //template <typename T>
    //void report_error( const std::string& s, T err_code, uint32_t fail_num );
    //==================================================
    

public:
    Logger logger_;

    ZynqDetector();
    ~ZynqDetector() = default;

    //void DummyDetector::create_irq_task_map();

    void set_zynq( std::unique_ptr<DerivedZynq> z );
    void set_network( std::unique_ptr<DerivedNetwork> n );
    
    void network_init();
    void queue_init();
    //void interrupt_init();
    void task_init();
    void create_network_tasks();
    void create_device_access_tasks();
};

#include "ZynqDetector.tpp"
