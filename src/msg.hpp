#pragma once

#include <cstdint>

typedef uint16_t msg_id_t;

const uint16_t MAX_UDP_MSG_LENG = 4096;
const uint16_t MAX_UDP_MSG_DATA_LENG = MAX_UDP_MSG_LENG - 4; // length of message data in bytes

typedef uint16_t msg_t;
const msg_t MSG_READ_VER = 0;

// 2-bit message op field
const uint8_t MSG_OP_WR = 0;  // write a register/device
const uint8_t MSG_OP_RD = 1;  // read a register/device
const uint8_t MSG_OP_SET_POLL_PERIOD = 2; // set polling period
const uint8_t MSG_OP_ADD_TO_POLL = 3;     // add a register/device to polling list


typedef struct {
    uint16_t hdr;
    uint16_t op;
    uint32_t msg_data[MAX_UDP_MSG_DATA_LENG / 4];
} udp_msg_t;

typedef struct {
    uint32_t leng;
    uint32_t cfg_data[MAX_UDP_MSG_DATA_LENG / 4 - 1];
} asic_cfg_t;