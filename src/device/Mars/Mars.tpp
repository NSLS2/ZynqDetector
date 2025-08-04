#include "Register.hpp"


//===============================================================
// Constructor.
//===============================================================
template<typename DerivedNetwork>
Mars<DerivedNetwork>::Mars( Register&            reg
                          , QueueHandle_t const  mars_access_req_queue
                          )
                          : reg_       ( reg                   )
                          , req_queue_ ( mars_access_req_queue )
{}
//===============================================================


//===============================================================
// Stuff Mars by writing loads.
//===============================================================
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
//===============================================================


//===============================================================
// Create Mars task.
//===============================================================
template<typename DerivedNetwork>
void Mars<DerivedNetwork>::create_device_access_tasks()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { mars_cfg_task(); });
    xTaskCreate( task_wrapper
               , "MARS Cfg"
               , 1000
               , &task_func
               , 1
               , NULL
               );
}


//===============================================================
// MARS task definition.
//===============================================================
template<typename DerivedNetwork>
void Mars<DerivedNetwork>::mars_cfg_task()
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
//===============================================================

