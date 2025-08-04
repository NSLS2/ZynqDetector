

In Detector:



In GermaniumDetector:









//======================================================================//
// Register
//======================================================================//

// Register

template< typename DerivedRegister >
Register<DerivedRegister>::Register
    (   
      uintptr_t                       base_addr
    , const QueueHandle_t             single_access_req_queue
    , const QueueHandle_t             single_access_resp_queue
    , const Logger<DerivedRegister>&  logger
    )   
    : base_addr_                 ( base_addr                )
    , single_access_req_queue_   ( single_access_req_queue  )
    , single_access_resp_queue_  ( single_access_resp_queue )
    , logger_                    ( logger                   )
{}

// GermaniumRegister

GermaniumRegister::GermaniumRegister( uintptr_t                         base_addr
                                    , const QueueHandle_t               single_access_req_queue
                                    , const QueueHandle_t               single_access_resp_queue
                                    , const QueueHandle_t               multi_access_req_queue
                                    , const QueueHandle_t               multi_access_resp_queue
                                    , const Logger<GermaniumRegister>&  logger
                                    )
                                    : Register<GermaniumRegister> ( base_addr
                                                                  , single_access_req_queue
                                                                  , single_access_resp_queue
                                                                  , logger
                                                                  )
                                    , base_ ( static_cast<Register<GermaniumRegister>*>(this))
                                    , multi_access_req_queue  ( multi_access_req_queue  )
                                    , multi_access_resp_queue ( multi_access_resp_queue )
{}

//======================================================================//
// Zynq
//======================================================================//

// Zynq

std::unique_ptr<DerivedRegister> reg_;

template < typename DerivedZynq
         , typename DerivedRegister
         >
void Zynq<DerivedZynq, DerivedRegister>::set_register( std::unique_ptr<DerivedRegister> z )
{
    reg_ = std::move(z);
}

template < typename DerivedZynq
         , typename DerivedRegister
         >
Zynq<DerivedZynq, DerivedRegister>::Zynq
    ( uintptr_t                      base_addr
    , const QueueHandle_t            register_single_access_req_queue
    , const QueueHandle_t            register_single_access_resp_queue
    , const Logger<DerivedRegister>& logger
    )   
    : logger_ ( logger )
{}

// GermaniumZynq

GermaniumZynq::GermaniumZynq()
{
    auto r = std::make_unique<GermaniumRegister>(
        register_single_access_req_queue,
        ...
    );
    this->set_register(std::move(r));
}

GermaniumZynq::GermaniumZynq
    ( const QueueHandle_t              register_single_access_req_queue
    , const QueueHandle_t              register_single_access_resp_queue
    , const QueueHandle_t              register_multi_access_req_queue
    , const QueueHandle_t              register_multi_access_resp_queue
    , const QueueHandle_t              psi2c0_req_queue
    , const QueueHandle_t              psi2c0_resp_queue
    , const QueueHandle_t              psi2c1_req_queue
    , const QueueHandle_t              psi2c1_resp_queue
    , const QueueHandle_t              psxadc_req_queue
    , const QueueHandle_t              psxadc_resp_queue
    , const Logger<GermaniumRegister>& logger
    )
    : Zynq< GermaniumZynq
          , GermaniumRegister
          >
          ( XPAR_IOBUS_0_BASEADDR
          , register_single_access_req_queue
          , register_single_access_resp_queue
          , logger
          )
    , base_  ( static_cast<Zynq<GermaniumZynq, GermaniumRegister>*>(this) )
    , psi2c0_( std::make_unique<PsI2c<GermaniumRegister>>( 0
                                      , "psi2c0"
                                      , XPAR_I2C0_BASEADDR
                                      , XPAR_I2C0_CLOCK_FREQ
                                      , psi2c0_req_queue
                                      , psi2c0_resp_queue
                                      , logger
                                      )
             )
    , psi2c1_( std::make_unique<PsI2c<GermaniumRegister>>( 1
                                      , "psi2c1"
                                      , XPAR_I2C1_BASEADDR
                                      , XPAR_I2C1_CLOCK_FREQ
                                      , psi2c1_req_queue
                                      , psi2c1_resp_queue
                                      , logger
                                      )
             )
    , psxadc_( std::make_unique<PsXadc<GermaniumRegister>>( "psxadc"
                                       , psxadc_req_queue
                                       , psxadc_resp_queue
                                       , logger
                                       )
             )
    , logger_ ( logger )
{
    auto r = std::make_unique<GermaniumRegister>( XPAR_IOBUS_0_BASEADDR
                                                , register_single_access_req_queue
                                                , register_single_access_resp_queue
                                                , register_multi_access_req_queue
                                                , register_multi_access_resp_queue
                                                , logger
                                                );
    this->set_register( std::move(r) );
}

//======================================================================//

std::unique_ptr<DerivedZynq> zynq_;

template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        , typename DerivedRegister
        >
void ZynqDetector< DerivedDetector
            , DerivedNetwork
            , DerivedZynq
            , DerivedRegister
            >::set_zynq(std::unique_ptr<DerivedZynq> z)
{
    zynq_ = std::move(z);
}

template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        , typename DerivedRegister
        >
void ZynqDetector< DerivedDetector
            , DerivedNetwork
            , DerivedZynq
            , DerivedRegister
            >::set_zynq(std::unique_ptr<DerivedZynq> n)
{
    network_ = std::move(n);
}

template< typename DerivedDetector
        , typename DerivedNetwork
        , typename DerivedZynq
        , typename DerivedRegister
        >
ZynqDetector< DerivedDetector
            , DerivedNetwork
            , DerivedZynq
            , DerivedRegister
            >::ZynqDetector()
//    : zynq_    ()
//    , network_ ( std::move(net) )
{
}

//======================================================================//

GermaniumDetector::GermaniumDetector()
    : ZynqDetector< GermaniumDetector
                  , GermaniumNetwork
                  , GermaniumZynq
                  , GermaniumRegister
                  >()
    , ltc2309_0_ ( std::make_unique<Ltc2309<PsI2c, PsI2cAccessReq>>( psi2c_1
                                                   , Ltc2309_0_I2C_ADDR
                                                   , true
                                                   , psi2c1_req_queue_
                                                   , chan_assign
                                                   , this->logger_
                                                   )
                 )
    , ltc2309_1_ ( std::make_unique<Ltc2309<PsI2c, PsI2cAccessReq>>( psi2c_1
                                                   , Ltc2309_1_I2C_ADDR
                                                   , true
                                                   , psi2c1_req_queue_
                                                   , chan_assign
                                                   , this->logger_
                                                   )
                 )
    , dac7678_   ( std::make_unique<Dac7678<PsI2c, PsI2cAccessReq>>( psi2c_1
                                                   , Dac7678_I2C_ADDR
                                                   , psi2c1_req_queue_
                                                   , chan_assign
                                                   , this->logger_
                                                   )
                 )
    , tmp100_0_  ( std::make_unique<Tmp100<PsI2c, PsI2cAccessReq>>( psi2c_0
                                                  , Tmp100_0_I2C_ADDR
                                                  , psi2c0_req_queue_
                                                  , this->logger_
                                                  )
                 )
    , tmp100_1_  ( std::make_unique<Tmp100<PsI2c, PsI2cAccessReq>>( psi2c_0
                                                  , Tmp100_1_I2C_ADDR
                                                  , psi2c0_req_queue_
                                                  , this->logger_
                                                  )
                 )
    , tmp100_2_  ( std::make_unique<Tmp100<PsI2c, PsI2cAccessReq>>( psi2c_0
                                                  , Tmp100_2_I2C_ADDR
                                                  , psi2c0_req_queue_
                                                  , this->logger_
                                                  )
                 )
{
    psi2c0_req_queue = xQueueCreate( 5, sizeof(PsI2cAccessReq) );
    psi2c1_req_queue = xQueueCreate( 5, sizeof(PsI2cAccessReq) );
    psxadc_req_queue = xQueueCreate( 5, sizeof(PsXadcAccessReq) );

    psi2c0_resp_queue = xQueueCreate( 5, sizeof(PsI2cAccessResp) );
    psi2c1_resp_queue = xQueueCreate( 5, sizeof(PsI2cAccessResp) );
    psxadc_resp_queue = xQueueCreate( 5, sizeof(PsXadcAccessResp) );

    resp_queue_set = xQueueCreateSet(50);

    xQueueAddToSet( psi2c0_resp_queue, resp_queue_set );
    xQueueAddToSet( psi2c1_resp_queue, resp_queue_set );
    xQueueAddToSet( psxadc_resp_queue, resp_queue_set );


    auto z = std::make_unique<GermaniumZynq>( register_single_access_req_queue_
                                            , register_single_access_resp_queue_
                                            , register_multi_access_req_queue_
                                            , register_multi_access_resp_queue_
                                            , psi2c0_req_queue_
                                            , psi2c0_resp_queue_
                                            , psi2c1_req_queue_
                                            , psi2c1_resp_queue_
                                            , psxadc_req_queue_
                                            , psxadc_resp_queue_
                                            , this->logger_
                                            );

    this->set_zynq ( std::move(z) );

    auto n = std::make_unique<GermaniumNetwork>( register_single_access_req_queue_
                                               , register_single_access_resp_queue_
                                               , register_multi_access_req_queue_
                                               , register_multi_access_resp_queue_
                                               , psi2c0_req_queue_
                                               , psi2c0_resp_queue_
                                               , psi2c1_req_queue_
                                               , psi2c1_resp_queue_
                                               , psxadc_req_queue_
                                               , psxadc_resp_queue_
                                               , this->logger_
                                               );

    this->set_network( std::move(z) );

    network_->network_init();
}



//======================================================================//