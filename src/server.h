// MIT License
// Copyright (c) 2026 Seung-Sub
// See LICENSE.md for the full notice.
//
// TCP binary protocol server for art_gripper_daemon. Each request handler
// mutates the shared RobotData struct using the same locking discipline as
// the original HMC ROS service handlers.
#pragma once
#include <atomic>
#include <thread>
#include <vector>
#include <cstdint>
#include <memory>
#include "robotData.h"

class ArtGripperServer {
public:
    ArtGripperServer(std::shared_ptr<RobotData> robot_data,
                     uint16_t port,
                     const std::string& bind_addr = "0.0.0.0");
    ~ArtGripperServer();

    bool Start();
    void Stop();

    // Set by SIGTERM/SIGINT handler in main, observed by daemon to begin shutdown.
    std::atomic<bool>* exit_flag() { return &_exit_flag; }

private:
    void ServerLoop();
    void HandleClient(int client_fd);

    std::shared_ptr<RobotData> _robot_data;
    uint16_t _port;
    std::string _bind_addr;
    int _listen_fd = -1;
    std::thread _server_thread;
    std::atomic<bool> _running{false};
    std::atomic<bool> _exit_flag{false};
};
