/**
 * @file GermaniumDetector.hpp
 * @brief Class definition of `GermaniumDetector`.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 08/11/2025
 * @copyright
 * Copyright (c) 2025 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <map>

#include "ZynqDetector.hpp"
#include "GermaniumNetwork.hpp"
#include "GermaniumZynq.hpp"

#include "Ltc2309.hpp"
#include "PsI2c.hpp"
#include "Dac7678.hpp"
#include "Tmp100.hpp"
#include "Ad9252.hpp"
#include "Mars.hpp"
#include "Zddm.hpp"

//===========================================================================//

class GermaniumDetector : public ZynqDetector< GermaniumDetector
                                             , GermaniumNetwork
                                             , GermaniumZynq
                                             >
{

protected:
    TaskHandle_t   psi2c_0_task_handler_;
    TaskHandle_t   psi2c_1_task_handler_;
    TaskHandle_t   psxadc_task_handler_;
    TaskHandle_t   ad9252_task_handler_;

    QueueHandle_t  psi2c_0_access_req_queue_;
    QueueHandle_t  psi2c_1_access_req_queue_;
    QueueHandle_t  psxadc_access_req_queue_;
    QueueHandle_t  ad9252_access_req_queue_;
    QueueHandle_t  mars_access_req_queue_;
    QueueHandle_t  zddm_access_req_queue_;

    QueueHandle_t  psi2c_access_resp_queue_;
    QueueHandle_t  psxadc_access_resp_queue_;

    std::unique_ptr<Ltc2309<PsI2c>> ltc2309_0_, ltc2309_1_;
    std::unique_ptr<Dac7678<PsI2c>> dac7678_;
    std::unique_ptr<Tmp100<PsI2c>>  tmp100_0_;
    std::unique_ptr<Tmp100<PsI2c>>  tmp100_1_;
    std::unique_ptr<Tmp100<PsI2c>>  tmp100_2_;

    std::unique_ptr<Ad9252<GermaniumNetwork>>  ad9252_;
    std::unique_ptr<Mars<GermaniumNetwork>>    mars_;
    std::unique_ptr<Zddm<GermaniumNetwork>>    zddm_;

    //int num_chips_;
    //int nelm_;

    ///< ASIC configuration data
    alignas(32) uint16_t loads[12][14];
  
    ///< I2C device addresses
    const uint8_t LTC2309_0_I2C_ADDR = 0x08;
    const uint8_t LTC2309_1_I2C_ADDR = 0x0A;
    const uint8_t DAC7678_I2C_ADDR   = 0x1A;
    const uint8_t TMP100_0_I2C_ADDR  = 0x48;
    const uint8_t TMP100_1_I2C_ADDR  = 0x49;
    const uint8_t TMP100_2_I2C_ADDR  = 0x59;

    const uint16_t VL0 = 0;
    const uint16_t VL1 = 1;
    const uint16_t VH1 = 2;
    const uint16_t VL2 = 3;
    const uint16_t VH2 = 4;
    const uint16_t HV  = 5;
    const uint16_t P1  = 6;
    const uint16_t P2  = 7;

    const uint16_t TEMP1   = 0;
    const uint16_t TEMP2   = 1;
    const uint16_t TEMP3   = 2;
    const uint16_t TEMP4   = 3;
    const uint16_t HV_RBV  = 4;
    const uint16_t HV_CURR = 5;
    //const uint16_t P1      = 
    //const uint16_t P2      = 
    const uint16_t ILEAK   = 8;
    const uint16_t P_V     = 9;

    //const uint16_t TEMP1 = 
    //const uint16_t TEMP2 = 
    //const uint16_t TEMP3 = 

    ///< DAC process map: <op, <device, channel>>
    const std::map<uint16_t, std::pair<Dac7678<PsI2c>*, uint8_t>> dac7678_instr_map_ =
        {
            { VL0, std::make_pair( dac7678_.get(), 0 ) },
            { VL1, std::make_pair( dac7678_.get(), 1 ) },
            { VH1, std::make_pair( dac7678_.get(), 2 ) },
            { VL2, std::make_pair( dac7678_.get(), 3 ) },
            { VH2, std::make_pair( dac7678_.get(), 4 ) },
            { HV,  std::make_pair( dac7678_.get(), 5 ) },
            { P1,  std::make_pair( dac7678_.get(), 6 ) },
            { P2,  std::make_pair( dac7678_.get(), 7 ) }
        };

    ///< ADC process map: <op, <device, channel>>
    const std::map<uint16_t, std::pair<Ltc2309<PsI2c>*, uint8_t>> ltc2309_instr_map_ =
        {
            { TEMP1,   std::make_pair( ltc2309_0_.get(), 0 ) },
            { TEMP2,   std::make_pair( ltc2309_0_.get(), 1 ) },
            { TEMP3,   std::make_pair( ltc2309_0_.get(), 2 ) },
            { TEMP4,   std::make_pair( ltc2309_0_.get(), 3 ) },
            { HV_RBV,  std::make_pair( ltc2309_0_.get(), 4 ) },
            { HV_CURR, std::make_pair( ltc2309_0_.get(), 5 ) },
            { P1,      std::make_pair( ltc2309_0_.get(), 6 ) },
            { P2,      std::make_pair( ltc2309_0_.get(), 7 ) },
            { ILEAK,   std::make_pair( ltc2309_1_.get(), 0 ) },
            { P_V,     std::make_pair( ltc2309_1_.get(), 1 ) }
        };

    ///< Temperature process map: <op, <device, channel>>
    const std::map<uint16_t, Tmp100<PsI2c>*> tmp100_instr_map_ =
        {
            { TEMP1, tmp100_0_.get() },
            { TEMP2, tmp100_1_.get() },
            { TEMP3, tmp100_2_.get() }
        };

    using I2cAccessHandler = std::function<void(const UdpRxMsg&)>;
    std::map<uint16_t, I2cAccessHandler> i2c_access_dispatch_map_;

    void init_i2c_access_dispatch_map();

    void rx_msg_proc(const UdpRxMsg& udp_msg);

    //void polling_taski_init();

public:
    static constexpr size_t REGISTER_SINGLE_ACCESS_REQ_QUEUE_LENG  = 100;
    static constexpr size_t REGISTER_SINGLE_ACCESS_RESP_QUEUE_LENG = 100;

    ///< Pointer pointing to base/ZynqDetector
    ZynqDetector< GermaniumDetector
                , GermaniumNetwork
                , GermaniumZynq
                >* base_;

    GermaniumDetector();

    /**
     * @brief Create tasks to acess GermaniumDetector specific devices.
     */
    void create_device_access_tasks_special();

    /**
     * @brief Create GermaniumDetector specific queues.
     */
    void create_queues_special();

    /**
     * @brief Create GermaniumDetector specific components.
     */
    void create_components_special();
};

//===========================================================================//

