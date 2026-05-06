// art_gripper_standalone — ROS-free daemon entry.
// Mirrors gripper_pier_ws/src/art_gripper/src/gripper_ecat/main.cpp but replaces
// rclcpp::spin(GripperEcat) with our TCP binary protocol server.
#include <signal.h>
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#include <cstdlib>
#include <cstring>

#include "sysData.h"
#include "robotData.h"
#include "thread/threadMan.h"
#include "device/deviceMan.h"
#include "server.h"
#include "protocol.h"

// Global instances — same lifetime model as the ROS variant so the existing
// device/thread code paths see identical structures.
std::shared_ptr<RobotData> g_robot_data = std::make_shared<RobotData>();
std::shared_ptr<SysData>   g_sys_data   = std::make_shared<SysData>();
static ArtGripperServer*   g_server     = nullptr;

static void CatchSignal(int) {
    g_sys_data->run = false;
    if (g_server) g_server->Stop();
}

int main(int argc, const char* argv[]) {
    signal(SIGTERM, CatchSignal);
    signal(SIGINT,  CatchSignal);

    g_sys_data->robotData = g_robot_data.get();
    g_sys_data->ecMasterIndex = 0;
    uint16_t port = art_gripper::DEFAULT_PORT;
    std::string bind_addr = "0.0.0.0";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "-m" || a == "--master") && i+1 < argc) {
            g_sys_data->ecMasterIndex = std::atoi(argv[++i]);
        } else if (a == "--port" && i+1 < argc) {
            port = (uint16_t)std::atoi(argv[++i]);
        } else if (a == "--bind" && i+1 < argc) {
            bind_addr = argv[++i];
        } else if (a == "--help" || a == "-h") {
            std::cout << "Usage: art_gripper_daemon [-m <master>] [--port <p>] [--bind <ip>]\n";
            return 0;
        }
    }

    std::cout << "[art_gripper] EtherCAT master index: " << g_sys_data->ecMasterIndex << "\n";
    std::cout << "[art_gripper] bind: " << bind_addr << ":" << port << "\n";

    if (initDevice(g_sys_data.get())) {
        std::cerr << "initDevice failed\n";
        return 1;
    }
    if (initThread(g_sys_data.get())) {
        std::cerr << "initThread failed\n";
        closeDevice(g_sys_data.get());
        return 1;
    }

    ArtGripperServer server(g_robot_data, port, bind_addr);
    g_server = &server;
    if (!server.Start()) {
        std::cerr << "server start failed\n";
        closeThread();
        closeDevice(g_sys_data.get());
        return 1;
    }

    // Spin until SIGINT/SIGTERM or OP_SHUTDOWN flips the exit flag.
    while (g_sys_data->run && !server.exit_flag()->load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "[art_gripper] shutting down...\n";
    server.Stop();
    closeThread();
    closeDevice(g_sys_data.get());
    return 0;
}
