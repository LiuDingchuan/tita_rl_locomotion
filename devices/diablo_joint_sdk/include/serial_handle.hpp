
#pragma once

#include <thread>
#include <vector>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include "crc_verify.hpp"
#include "joint_ctrl_protocol.h"
#include "VulcanSerial/SerialPort.hpp"
#include <mutex>
#include <memory>

using namespace std;

class SerialHandle
{
private:
    bool rec_loop;
    uint8_t rec_buffer[2][100]; // 双缓冲
    int write_idx = 0;          // 当前写入buffer的索引
    int read_idx = 1;           // 当前读取buffer的索引
    std::mutex swap_mutex;      // 控制索引切换
    VulcanSerial::SerialPort mySerial;
    std::shared_ptr<std::thread> serial_rx_thd;
    uint16_t send_cnt = 0;

private:
    void serial_recive(void);

public:
    std::shared_ptr<uart_packet_t> rec_package;

public:
    uint16_t receive_cnt = 0;
    void send_commond(const motor_torque_t &ctrl_package);
    void serial_init(const std::string dev = "/dev/ttyTHS2");
    void create_package(_Float32 *motor_tor, motor_torque_t &motor_package);
    void start_joint_sdk(void);
    void get_latest_packet(uart_packet_t &package);
    SerialHandle(/* args */) {}
    ~SerialHandle();
};
