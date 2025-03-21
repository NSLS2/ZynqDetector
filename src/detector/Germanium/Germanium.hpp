
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

class Germanium : public ZynqDetector<Germanium>
{
private:
    const uint32_t base_addr_ = 0x43C00000;

    int num_chips_;
    int nelm_;

    // Mars configuration variables
    // struct chipstr
    // {
    //     unsigned int pa : 10;     /* Threshold dac */
    //     unsigned int pb : 10;     /* Test pulse dac */
    //     unsigned char rm : 1;     /* Readout mods; 1=synch, 0=asynch */
    //     unsigned char senfl1 : 1; /* Lock on peak found */
    //     unsigned char senfl2 : 1; /* Lock on threshold */
    //     unsigned char m0 : 1;     /* 1=channel mon, 0=others */
    //     unsigned char m1 : 1;     /* 1=pk det on PD/PN; 0=other mons on PD/PN */
    //     unsigned char sbn : 1;    /* enable buffer on pdn & mon outputs */
    //     unsigned char sb : 1;     /* enable buffer on pd & mon outputs */
    //     unsigned char sl : 1;     /* 0=internal 2pA leakage, 1=disabled */
    //     unsigned char ts : 2;     /* Shaping time */
    //     unsigned char rt : 1;     /* 1=timing ramp duration x 3 */
    //     unsigned char spur : 1;   /* 1=enable pileup rejector */
    //     unsigned char sse : 1;    /* 1=enable multiple-firing suppression */
    //     unsigned char tr : 2;     /* timing ramp adjust */
    //     unsigned char ss : 2;     /* multiple firing time adjust */
    //     unsigned char c : 5;      /* m0=0,Monitor select. m0=1, channel being monitored */
    //     unsigned char g : 3;      /* Gain select */
    //     unsigned char slh : 1;    /* internal leakage adjust */
    //     unsigned char sp : 1;     /* Input polarity; 1=positive, 0=negative */
    //     unsigned char saux : 1;   /* Enable monitor output */
    //     unsigned char sbm : 1;    /* Enable output monitor buffer */
    //     unsigned char tm : 1;     /* Timing mode; 0=ToA, 1=ToT */
    // }; // 48-bit
    // volatile struct chipstr globalstr[12];

    // struct chanstr
    // {
    //     unsigned char dp : 4;  /* Pileup rejector trim dac */
    //     unsigned char nc1 : 1; /* no connection, set 0 */
    //     unsigned char da : 3;  /* Threshold trim dac */
    //     unsigned char sel : 1; /* 1=leakage current, 0=shaper output */
    //     unsigned char nc2 : 1; /* no connection, set 0 */
    //     unsigned char sm : 1;  /* 1=channel disable */
    //     unsigned char st : 1;  /* 1=enable test input (30fF cap) */
    // }; // 12 bit

    // volatile struct chanstr channelstr[384];
    unsigned int loads[12][14];

    typedef union 
    {
        
    } GermaniumConfigReq;
    

    void latch_conf();
    void update_loads( char* loads );
    void send_spi_bit( int chip_sel, int val );
    void load_ad9252reg( int chip_sel, int addr, int data );
    int  ad9252_cfg( int chip_num, int addr, int data );
    void zddm_arm( int mode, int val );



    typedef struct 
    {
        uint16_t op;
        
    } RegisterMultiAccessReq;

    Zynq zynq_;
    PSI2C &psi2c_0_;
    PSI2C &psi2c_1_;
    PSXADC psxadc_;
    LTC2309<PSI2C> ltc2309_;
    DAC7678<PSI2C> dac7678_;
    TMP100<PSI2C> tmp100_0_;
    TMP100<PSI2C> tmp100_1_;
    TMP100<PSI2C> tmp100_2_;

    unsigned int loads_[12][14];

    const uint8_t LTC2309_I2C_ADDR = 72;
    const uint8_t DAC7678_I2C_ADDR = 72;
    const uint8_t TMP100_0_I2C_ADDR = 72;
    const uint8_t TMP100_1_I2C_ADDR = 72;
    const uint8_t TMP100_2_I2C_ADDR = 72;

    TaskHandle_t psi2c_0_task_handler_;
    TaskHandle_t psi2c_1_task_handler_;
    TaskHandle_t psxadc_task_handler_;

    QueueHandle_t psi2c_0_req_queue;
    QueueHandle_t psi2c_1_req_queue;
    QueueHandle_t psxadc_req_queue;

    QueueHandle_t psi2c_0_resp_queue;
    QueueHandle_t psi2c_1_resp_queue;
    QueueHandle_t psxadc_resp_queue;

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

    /*
    static void udp_tx_task_wrapper(void* param)
    {
        auto obj = static_cast<Germanium*>(param);  // get `this` of Germanium
        obj->udp_tx_task();
    }
    */

protected:
    // void greate_tasks();

    void rx_msg_proc(const udt_msg_t &udp_msg);
    void tx_msg_proc();

    void ps_i2c_access_task();
    void ps_xadc_access_task();

public:
    Germanium();
    void task_init() override;
};
