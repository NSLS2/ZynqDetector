
//=======================================================================
//=======================================================================

template<typename DerivedNetwork>
Ad9252<DerivedNetwork>::Ad9252( Register&            reg
                              , QueueHandle_t const  ad9252_access_req_queue
                              )
                              : reg_              ( reg                     )
                              , req_queue_ ( ad9252_access_req_queue )
{}
//=======================================================================


//=======================================================================
//=======================================================================
template<typename DerivedNetwork>
void Ad9252<DerivedNetwork>::set_clk_skew ( int chip_num, int skew )
{
    ad9252_cnfg( chip_num, 22, skew );  /* clock skew adjust */
    ad9252_cnfg( chip_num, 255, 1 ); /* latch regs */
}
//=======================================================================


//=======================================================================
//=======================================================================
template<typename DerivedNetwork>
void Ad9252<DerivedNetwork>::ad9252_cnfg( int chip_num, int addr, int val )
{
    int chip_sel;

    switch( chip_num )
    {
        case 1:
            chip_sel = 0b11000;
            break;
        case 2:
            chip_sel = 0b10100;
            break;
        case 3:
            chip_sel = 0b01100;
            break;
        default:
            chip_sel = 0b00000;
    }
    reg_.multi_access_start();
    
    reg_.multi_access_write( DerivedNetwork::ADC_SPI, chip_sel );
    
    load_reg( chip_sel, addr, val );
    
    reg_.multi_access_write( DerivedNetwork::ADC_SPI, 0b11100 );

    reg_.multi_access_end();
}
//=======================================================================


//=======================================================================
//=======================================================================
template<typename DerivedNetwork>
void Ad9252<DerivedNetwork>::load_reg( int chip_sel, int addr, int data )
{
    int i;

    // small delay
    for (i = 0; i < 100; i++)
        ;

    // Read/Write bit
    send_spi_bit( chip_sel, 0 );

    // W1=W0=0 (word length = 1 byte)
    for (i = 1; i >= 0; i--)
        send_spi_bit( chip_sel, 0 );

    // address
    for (i = 12; i >= 0; i--)
        send_spi_bit( chip_sel, addr >> i );

    // data
    for (i = 7; i >= 0; i--)
        send_spi_bit( chip_sel, data >> i );

    // small delay
    for (i = 0; i < 100; i++)
        ;
}
//=======================================================================


//=======================================================================
//=======================================================================
template<typename DerivedNetwork>
void Ad9252<DerivedNetwork>::send_spi_bit( int chip_sel, int val )
{
    int sda;

    sda = val & 0x1;

    // set sclk low
    reg_.multi_access_write( DerivedNetwork::ADC_SPI, (chip_sel | 0) );
    // set data with clock low
    reg_.multi_access_write( DerivedNetwork::ADC_SPI, (chip_sel | sda) );
    // set clk high
    reg_.multi_access_write( DerivedNetwork::ADC_SPI, (chip_sel | 0x2 | sda) );
    // set clk low
    reg_.multi_access_write( DerivedNetwork::ADC_SPI, (chip_sel | sda) );
    // set data low
    reg_.multi_access_write( DerivedNetwork::ADC_SPI, (chip_sel | 0) );

}
//=======================================================================


//=======================================================================
//=======================================================================
template<typename DerivedNetwork>
void Ad9252<DerivedNetwork>::create_device_access_tasks()
{
    auto task_func = std::make_unique<std::function<void()>>([this]() { ad9252_cfg_task(); });
    xTaskCreate( task_wrapper
               , "AD9252 Cfg"
               , 1000
               , &task_func
               , 1
               , NULL
               );
}
//=======================================================================


//=======================================================================
//=======================================================================
template<typename DerivedNetwork>
void Ad9252<DerivedNetwork>::ad9252_cfg_task()
{
    Ad9252AccessReq  req;

    while(1)
    {
        xQueueReceive( req_queue_
                     , &req
                     , portMAX_DELAY
                     );

        set_clk_skew( req.chip_num, req.data );
    }
}
//=======================================================================
