/**
 * @file Ad9252.cpp
 * @brief Member function definitions of `Ad9252`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */

//===========================================================================//

/**
 * @brief Ad9252 constructor.
 * @param reg Reference to the register.
 * @param ad9252_access_req_queue Queue for passing AD9252 access requests.
 */
template<typename DerivedNetwork>
Ad9252<DerivedNetwork>::Ad9252( Register&            reg
                              , QueueHandle_t const  ad9252_access_req_queue
                              )
                              : reg_              ( reg                     )
                              , req_queue_ ( ad9252_access_req_queue )
{}

//===========================================================================//

/**
 * @brief Set clock skew.
 * @param chipnum Chip (AD9252) number.
 * @param skew Clock skew.
 */
template<typename DerivedNetwork>
void Ad9252<DerivedNetwork>::set_clk_skew ( int chip_num, int skew )
{
    ad9252_cnfg( chip_num, 22, skew );  /* clock skew adjust */
    ad9252_cnfg( chip_num, 255, 1 ); /* latch regs */
}

//===========================================================================//

/**
 * @brief Configure AD9252. Currently it sets clock skew.
 * @param chip_num Chip (AD9252) number.
 * @param addr Address.
 * @param val Configuration value. Currently it is clock skew.
 */
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
    
    reg_.write( DerivedNetwork::ADC_SPI, chip_sel );
    
    load_reg( chip_sel, addr, val );
    
    reg_.write( DerivedNetwork::ADC_SPI, 0b11100 );

}

//===========================================================================//

/**
 * @brief Load data to register
 * @param chip_sel Chip select value, generated from chip number.
 * @param addr Address.
 * @param data Data to be loaded to the register.
 */
template<typename DerivedNetwork>
void Ad9252<DerivedNetwork>::load_reg( int chip_sel, int addr, int data )
{
    int i;

    // small delay
    for (i = 0; i < 100; i++)
        ;

    reg_.multi_access_start();

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

    reg_.multi_access_end();

    // small delay
    for (i = 0; i < 100; i++)
        ;
}

//===========================================================================//

/**
 * @brief Send SPI bit.
 * @param chip_sel Chip select value.
 * @param val Value to be sent.
 */
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

//===========================================================================//

/**
 * @brief Create AD9252 access task.
 */
template<typename DerivedNetwork>
void Ad9252<DerivedNetwork>::create_device_access_tasks()
{
    task_cfg_ = { .entry = [](void* ctx) { static_cast<Ad9252<DerivedNetwork>*>(ctx)->task(); },
                  .context = this
                };

    xTaskCreateStatic( task_wrapper
               , "AD9252 Cfg"
               , TASK_STACK_SIZE
               , &task_cfg_
               , 1
               , task_stack_
               , &task_tcb_
               );
}

//===========================================================================//

/**
 * @brief AD9252 access task function.
 */
template<typename DerivedNetwork>
void Ad9252<DerivedNetwork>::task()
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

//===========================================================================//
