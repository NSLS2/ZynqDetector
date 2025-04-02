
#include <cstdint>
#include <string>
#include <map>

#include "ZynqDetector.hpp"




// const uint16_t LKUPADDRREG       = ?;

using Instruction_Handler = std::function<void()>;

struct germ_udp_msg_t
{
    uint32_t data;
};

class GermaniumDetector : public ZynqDetector<GermaniumDetector>
{

protected:
    // void greate_tasks();
    TaskHandle_t psi2c_0_task_handler_;
    TaskHandle_t psi2c_1_task_handler_;
    TaskHandle_t psxadc_task_handler_;

    QueueHandle_t psi2c_0_req_queue;
    QueueHandle_t psi2c_1_req_queue;
    QueueHandle_t psxadc_req_queue;

    QueueHandle_t psi2c_0_resp_queue;
    QueueHandle_t psi2c_1_resp_queue;
    QueueHandle_t psxadc_resp_queue;


    int num_chips_;
    int nelm_;

    // volatile struct chanstr channelstr[384];
    unsigned int loads[12][14];
  
    PSI2C &psi2c_0_;
    PSI2C &psi2c_1_;
    PSXADC psxadc_;
    LTC2309<PSI2C> ltc2309_;
    DAC7678<PSI2C> dac7678_;
    TMP100<PSI2C>  tmp100_0_;
    TMP100<PSI2C>  tmp100_1_;
    TMP100<PSI2C>  tmp100_2_;

    unsigned int loads_[12][14];

    const uint8_t LTC2309_I2C_ADDR = 72;
    const uint8_t DAC7678_I2C_ADDR = 72;
    const uint8_t TMP100_0_I2C_ADDR = 72;
    const uint8_t TMP100_1_I2C_ADDR = 72;
    const uint8_t TMP100_2_I2C_ADDR = 72;


    // QueueSetHandle_t resp_queue_set;

    //======================================
    // Instruction map
    //======================================
    const std::map<int, std::function<void()>> instruction_map {
        { MARS_CONF_LOAD, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
        { LEDS, [this]() { this->load_mars_conf(); } }
      };  

    void latch_conf();
    void update_loads( char* loads );
    void send_spi_bit( int chip_sel, int val );
    void load_ad9252reg( int chip_sel, int addr, int data );
    int  ad9252_cfg( int chip_num, int addr, int data );
    void zddm_arm( int mode, int val );

    void rx_msg_proc(const udt_msg_t &udp_msg);
    void tx_msg_proc();

    void ps_i2c_access_task();
    void ps_xadc_access_task();

public:
    GermaniumDetector();
    void task_init() override;
};
