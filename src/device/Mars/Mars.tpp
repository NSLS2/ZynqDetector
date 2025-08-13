/**
 * @file Mars.cpp
 * @brief Member function definitions of `Mars`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

#include "Register.hpp"

//===========================================================================//

/**
 * @brief Mars constructor.
 * @param reg Reference to the register.
 * @param mars_access_req_queue Queue for passing MARS access requests.
 */
template<typename DerivedNetwork>
Mars<DerivedNetwork>::Mars( Register&            reg
                          , QueueHandle_t const  mars_access_req_queue
                          )
                          : reg_       ( reg                   )
                          , req_queue_ ( mars_access_req_queue )
{}

//===========================================================================//

/**
 * @brief Stuff MARS by writing loads.
 */
template<typename DerivedNetwork>
void Mars<DerivedNetwork>::stuff_mars( const uint32_t (&loads)[12][14] )
{
    reg_.multi_access_start();

    for ( int i = 0; i < 12; i++ )
    {
        reg_.write( DerivedNetwork::MARS_CONF_LOAD, 4 );
        reg_.write( DerivedNetwork::MARS_CONF_LOAD, 0 );

        for ( int j = 0; j < 14; j++ )
        {
            reg_.write( DerivedNetwork::MARS_CONF_LOAD, loads[i][j] );

            //latch_conf();
            reg_.write( DerivedNetwork::MARS_CONF_LOAD, 2 );
            reg_.write( DerivedNetwork::MARS_CONF_LOAD, 0 );

            vTaskDelay(pdMS_TO_TICKS(1));;
        }

        reg_.write( DerivedNetwork::MARS_CONF_LOAD, 0x00010000 << i );
        reg_.write( DerivedNetwork::MARS_CONF_LOAD, 0 );

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    reg_.multi_access_end();
}

//===========================================================================//

/**
 * @brief Create MARS access task.
 */
template<typename DerivedNetwork>
void Mars<DerivedNetwork>::create_device_access_tasks()
{
    task_cfg_ = { .entry = [](void* ctx) { static_cast<Mars<DerivedNetwork>*>(ctx)->task(); },
                  .context = this
                };

    xTaskCreateStatic( task_wrapper
               , "MARS Cfg"
               , TASK_STACK_SIZE
               , &task_cfg_
               , 1
               , task_stack_
               , &task_tcb_
               );
}


//===========================================================================//

/**
 * @brief MARS access task function.
 */
template<typename DerivedNetwork>
void Mars<DerivedNetwork>::task()
{
    MarsAccessReq  req;

    while(1)
    {
        xQueueReceive( req_queue_
                     , &req
                     , portMAX_DELAY
                     );

        stuff_mars( req.loads );
    }
}

//===========================================================================//

