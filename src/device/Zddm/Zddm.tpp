/**
 * @file Zddm.cpp
 * @brief Member function definitions of `Zddm`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

/**
 * @brief Zddm constructor.
 * @param reg Reference to the register.
 * @param zddm_access_req_queue Queue for passing zddm access requests.
 */
template<typename DerivedNetwork>
Zddm<DerivedNetwork>::Zddm( Register&            reg
                          , QueueHandle_t const  zddm_access_req_queue
                          )
                          : reg_       ( reg                   )
                          , req_queue_ ( zddm_access_req_queue )
{}

//===========================================================================//

/**
 * @brief Arm zddm.
 * @param mode Operation mode.
 * @param val Value for TRIG.
 */
template<typename DerivedNetwork>
void Zddm<DerivedNetwork>::zddm_arm( int mode, int val )
{
    reg_.multi_access_start();

    if ( mode == 0 )
    {
        reg_.write( DerivedNetwork::TRIG, val );
    }
    else if ( mode == 1 )
    {
        if ( val == 0 )
            reg_.write( DerivedNetwork::TRIG, val );
    }

    reg_.multi_access_end();
}

//===========================================================================//

/**
 * @brief Create zddm access task.
 */
template<typename DerivedNetwork>
void Zddm<DerivedNetwork>::create_device_access_tasks()
{
    task_cfg_ = { .entry = [](void* ctx) { static_cast<Zddm<DerivedNetwork>*>(ctx)->task(); }
                , .context = this
                };

    xTaskCreateStatic( task_wrapper
               , "ZDDM Cfg"
               , TASK_STACK_SIZE
               , &task_cfg_
               , TASK_PRIORITY
               , task_stack_
               , &task_tcb_
               );
}

//===========================================================================//

/**
 * @brief Task function for zddm access.
 */
template<typename DerivedNetwork>
void Zddm<DerivedNetwork>::task()
{
    ZddmAccessReq  req;

    while(1)
    {
        xQueueReceive( req_queue_
                     , &req
                     , portMAX_DELAY
                     );

        zddm_arm( req.mode, req.val );
    }
}

//===========================================================================//
