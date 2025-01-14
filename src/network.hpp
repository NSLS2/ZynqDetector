#pragma once

#include <cstdint>
#include <array>

class network
{
private:
    std::array<uint8_t, 4> ip_addr;
    std::array<uint8_t, 4> netmask;
    std::array<uint8_t, 4> gateway;
    std::array<uint8_t, 4> dns;
    std::array<uint8_t, 6> mac_addr;

    void read_network_config( const std::string& filename );

public:
    network();
    static void udp_rx( void );
    static void udp_tx( void );
};
