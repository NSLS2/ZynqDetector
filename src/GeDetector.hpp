
#include <cstdint>
#include <string>
#include <map>

#include "ZynqDetector.hpp"

const uint16_t MARS_CONF_LOAD    = 0;
const uint16_t LEDS              = 1;
const uint16_t MARS_CONFIG       = 2;
const uint16_t VERSIONREG        = 3;
const uint16_t MARS_CALPULSE     = 4;
const uint16_t MARS_PIPE_DELAY   = 5;
const uint16_t MARS_RDOUT_ENB    = 8;
const uint16_t EVENT_TIME_CNTR   = 9;
const uint16_t SIM_EVT_SEL       = 10;
const uint16_t SIM_EVENT_RATE    = 11;
const uint16_t ADC_SPI           = 12;
const uint16_t CALPULSE_CNT      = 16;
const uint16_t CALPULSE_RATE     = 17;
const uint16_t CALPULSE_WIDTH    = 18;
const uint16_t CALPULSE_MODE     = 19;
const uint16_t TD_CAL            = 20;
const uint16_t EVENT_FIFO_DATA   = 24;
const uint16_t EVENT_FIFO_CNT    = 25;
const uint16_t EVENT_FIFO_CNTRL  = 26;
const uint16_t DMA_CONTROL       = 32;
const uint16_t DMA_STATUS        = 33;
const uint16_t DMA_BASEADDR      = 34;
const uint16_t DMA_BURSTLEN      = 35;
const uint16_t DMA_BUFLEN        = 36;
const uint16_t DMA_CURADDR       = 37;
const uint16_t DMA_THROTTLE      = 38;
const uint16_t UDP_IP_ADDR       = 40;
const uint16_t DMA_IRQ_THROTTLE  = 48;
const uint16_t DMA_IRQ_ENABLE    = 49;
const uint16_t DMA_IRQ_COUNT     = 50;
const uint16_t TRIG              = 52;
const uint16_t COUNT_TIME        = 53;
const uint16_t FRAME_NO          = 54;
const uint16_t COUNT_MODE        = 55;

//const uint16_t LKUPADDRREG       = ?;

using Instruction_Handler = std::function<void()>;



struct germ_udp_msg_t
{
    uint32_t data;
};

class GeRM: public ZynqDetector{
private:


    //======================================
    // Instruction map
    //======================================
    const std::map<int, std::function<void()>> instruction_map {
        { MARS_CONF_LOAD, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
        { LEDS, [this]() { this->load_mars_conf(); }
      }
    void greate_tasks();
    
    void rx_msg_proc( udt_msg_t udp_msg );
    void tx_msg_proc();
    
public:
    
};
