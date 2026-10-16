#pragma once

// C++ includes
#include <cstddef>
#include <iterator>
#include <portmacro.h>
#include <type_traits>
#include <memory>

// FreeRTOS includes
#include "FreeRTOS.h"
//#include "msg.hpp"
#include "task.h"
#include "queue.h"
#include "timers.h"

// Xilinx includes
#include "xil_printf.h"
#include "xparameters.h"

// Project includes
//#include "ZynqDetector.hpp"
//#include "pynq_ssd_msg.hpp"

#define TIMER_ID	1
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define TIMER_CHECK_THRESHOLD	9


///*
// * @brief ZynqDetector constructor.
// */
//template< typename DerivedDetector
//        , typename DerivedNetwork
//        , typename DerivedZynq
//        >
//ZynqDetector< DerivedDetector
//            , DerivedNetwork
//            , DerivedZynq
//            >::ZynqDetector()
//{
//}


/*
 * @brief Receive zynq_ from derived detector where it is created.
 * @param z std::unique_ptr<DerivedZynq>.
 */
template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::set_zynq(std::unique_ptr<DerivedZynq> z)
{
    zynq_ = std::move(z);
}

/*
 * @brief Receive network_ from derived detector where
 * it is created.
 * @param n std::unique_ptr<DerivedNetwork>.
 */
template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::set_network(std::unique_ptr<DerivedNetwork> n)
{
    network_ = std::move(n);
}

/*
 * @brief Initialize network from a configuration file.
 */
template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::network_init()
{
    network_->base_->network_init();
    //network_->network_init();
}

/*
 * @brief Initialize queues for request/response delivery.
 */
template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::create_queues()
{
    register_single_access_req_queue_ = xQueueCreate( DerivedDetector::REGISTER_SINGLE_ACCESS_REQ_QUEUE_LENG
                                                    , sizeof( RegisterSingleAccessReq )
                                                    );

    register_single_access_resp_queue_ = xQueueCreate( DerivedDetector::REGISTER_SINGLE_ACCESS_RESP_QUEUE_LENG
                                                     , sizeof( RegisterSingleAccessResp )
                                                     );

    static_cast<DerivedDetector*>(this)->create_queues_special();
}

/*
 * @brief Initialize all tasks.
 */
template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::create_device_access_tasks()
{
    zynq_->base_->create_device_access_tasks();
    static_cast<DerivedDetector*>(this)->create_device_access_tasks_special();
}

/*
 * @brief Initialize all tasks.
 */
template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::create_tasks()
{
    network_->base_->create_network_tasks();

    create_device_access_tasks();
}


/*
 * @brief Create components.
 */
template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        >
void ZynqDetector< DerivedDetector
                 , DerivedNetwork
                 , DerivedZynq
                 >::create_components()
{
    static_cast<DerivedDetector*>(this)->create_components_special();
}
