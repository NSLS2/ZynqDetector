template<typename DerivedNetwork>
Zddm<DerivedNetwork>::Zddm( Register&            reg
                          , QueueHandle_t const  zddm_access_req_queue
                          )
                          : reg_              ( reg                     )
                          , req_queue_ ( zddm_access_req_queue )
{}


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


template<typename DerivedNetwork>
void Zddm<DerivedNetwork>::create_device_access_tasks()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { zddm_cfg_task(); });
    xTaskCreate( task_wrapper
               , "ZDDM Cfg"
               , 1000
               , &task_func
               , 1
               , NULL
               );
}


template<typename DerivedNetwork>
void Zddm<DerivedNetwork>::zddm_cfg_task()
{
    ZddmAccessReq  req;

    while(1)
    {
        xQueueReceive( req_queue_
                     , &req
                     , portMAX_DELAY
                     );

        zddm_arm( 0, 0 );
    }
}

