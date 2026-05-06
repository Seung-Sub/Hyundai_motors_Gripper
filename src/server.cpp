#include "server.h"
#include "protocol.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <mutex>
#include <vector>

using namespace art_gripper;

static const char* INFO_NAME = "ART Gripper";
static const char* INFO_VERSION = "1.0.0";
static const char* INFO_DESC = "HMC ART Gripper standalone (ROS-free).";

ArtGripperServer::ArtGripperServer(std::shared_ptr<RobotData> robot_data,
                                   uint16_t port,
                                   const std::string& bind_addr)
    : _robot_data(robot_data), _port(port), _bind_addr(bind_addr) {}

ArtGripperServer::~ArtGripperServer() { Stop(); }

bool ArtGripperServer::Start() {
    _listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (_listen_fd < 0) { perror("socket"); return false; }
    int yes = 1;
    setsockopt(_listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(_port);
    if (inet_pton(AF_INET, _bind_addr.c_str(), &addr.sin_addr) != 1) {
        fprintf(stderr, "invalid bind addr %s\n", _bind_addr.c_str()); return false;
    }
    if (bind(_listen_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); return false;
    }
    if (listen(_listen_fd, 4) < 0) { perror("listen"); return false; }

    _running = true;
    _server_thread = std::thread(&ArtGripperServer::ServerLoop, this);
    fprintf(stdout, "[art_gripper] server listening on %s:%u\n", _bind_addr.c_str(), _port);
    return true;
}

void ArtGripperServer::Stop() {
    _running = false;
    if (_listen_fd >= 0) { ::shutdown(_listen_fd, SHUT_RDWR); ::close(_listen_fd); _listen_fd = -1; }
    if (_server_thread.joinable()) _server_thread.join();
}

void ArtGripperServer::ServerLoop() {
    while (_running) {
        sockaddr_in cli{}; socklen_t cl = sizeof(cli);
        int cfd = ::accept(_listen_fd, (sockaddr*)&cli, &cl);
        if (cfd < 0) { if (_running) perror("accept"); break; }
        int yes = 1;
        setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        // Serve client in caller thread (we expect single client at a time — GR00T env);
        // a simple pattern good enough for ~100 Hz commands.
        HandleClient(cfd);
        ::close(cfd);
    }
}

// Read exactly n bytes; return false on EOF/error.
static bool read_n(int fd, void* buf, size_t n) {
    auto* p = static_cast<uint8_t*>(buf);
    while (n > 0) {
        ssize_t r = ::recv(fd, p, n, 0);
        if (r <= 0) return false;
        p += r; n -= r;
    }
    return true;
}
static bool write_n(int fd, const void* buf, size_t n) {
    auto* p = static_cast<const uint8_t*>(buf);
    while (n > 0) {
        ssize_t w = ::send(fd, p, n, MSG_NOSIGNAL);
        if (w <= 0) return false;
        p += w; n -= w;
    }
    return true;
}
static bool send_response(int fd, uint8_t status, const void* payload, uint32_t plen) {
    if (!write_n(fd, &status, 1)) return false;
    uint32_t le = plen; // x86_64 already LE
    if (!write_n(fd, &le, 4)) return false;
    if (plen && !write_n(fd, payload, plen)) return false;
    return true;
}
static bool send_result_i32(int fd, int32_t r) {
    RespResult rr{r};
    return send_response(fd, STATUS_OK, &rr, sizeof(rr));
}
static bool send_err(int fd, uint8_t s) {
    return send_response(fd, s, nullptr, 0);
}

void ArtGripperServer::HandleClient(int fd) {
    while (_running) {
        uint8_t op; uint32_t plen;
        if (!read_n(fd, &op, 1)) return;
        if (!read_n(fd, &plen, 4)) return;
        if (plen > MAX_PAYLOAD) { send_err(fd, STATUS_INVALID_PAYLOAD); return; }
        std::vector<uint8_t> payload(plen);
        if (plen && !read_n(fd, payload.data(), plen)) return;

        switch (op) {
        case OP_PING: {
            const char* pong = "pong";
            send_response(fd, STATUS_OK, pong, 4);
            break;
        }
        case OP_GET_INFO: {
            uint8_t nlen = (uint8_t)strlen(INFO_NAME);
            uint8_t vlen = (uint8_t)strlen(INFO_VERSION);
            uint16_t dlen = (uint16_t)strlen(INFO_DESC);
            std::vector<uint8_t> buf;
            buf.push_back(nlen); buf.insert(buf.end(), INFO_NAME, INFO_NAME + nlen);
            buf.push_back(vlen); buf.insert(buf.end(), INFO_VERSION, INFO_VERSION + vlen);
            buf.push_back((uint8_t)(dlen & 0xff)); buf.push_back((uint8_t)(dlen >> 8));
            buf.insert(buf.end(), INFO_DESC, INFO_DESC + dlen);
            send_response(fd, STATUS_OK, buf.data(), (uint32_t)buf.size());
            break;
        }
        case OP_GET_STATUS: {
            RespStatus s{};
            {
                std::lock_guard<std::mutex> lk(_robot_data->mtx);
                s.gripper_status = _robot_data->status.gripper_status;
                s.finger_width   = _robot_data->status.finger_width;
                s.finger_pose    = _robot_data->status.finger_pose;
                for (int i=0;i<4;i++){
                    s.status_word[i] = _robot_data->status.status_word[i];
                    s.position[i]    = _robot_data->status.position[i];
                    s.velocity[i]    = _robot_data->status.velocity[i];
                    s.current[i]     = _robot_data->status.current[i];
                }
            }
            send_response(fd, STATUS_OK, &s, sizeof(s));
            break;
        }
        case OP_MOTOR_ON: {
            if (plen != sizeof(ReqMotorOn)) { send_err(fd, STATUS_INVALID_PAYLOAD); break; }
            auto* r = reinterpret_cast<ReqMotorOn*>(payload.data());
            std::lock_guard<std::mutex> lk(_robot_data->mtx);
            if (r->on) {
                _robot_data->control.gripper_control |= GRIPPER_COMMAND::SERVO_SWITCH_BIT;
                _robot_data->initProcess = 0;
            } else {
                _robot_data->control.gripper_control &= ~GRIPPER_COMMAND::SERVO_SWITCH_BIT;
            }
            send_result_i32(fd, 0);
            break;
        }
        case OP_SET_TARGET: {
            if (plen != sizeof(ReqSetTarget)) { send_err(fd, STATUS_INVALID_PAYLOAD); break; }
            auto* r = reinterpret_cast<ReqSetTarget*>(payload.data());
            std::lock_guard<std::mutex> lk(_robot_data->mtx);
            _robot_data->control.finger_width        = r->finger_width;
            _robot_data->control.finger_pose         = r->finger_pose;
            _robot_data->control.finger_width_speed  = r->finger_width_speed;
            _robot_data->control.finger_pose_speed   = r->finger_pose_speed;
            _robot_data->control.gripping_force      = r->gripping_force;
            _robot_data->control.contact_sensitivity = r->contact_sensitivity;
            _robot_data->control.gripper_control    |= GRIPPER_COMMAND::TARGET_UPDATE_BIT;
            send_result_i32(fd, 0);
            break;
        }
        case OP_SET_TARGET_WIDTH: {
            if (plen != sizeof(ReqU8)) { send_err(fd, STATUS_INVALID_PAYLOAD); break; }
            auto* r = reinterpret_cast<ReqU8*>(payload.data());
            std::lock_guard<std::mutex> lk(_robot_data->mtx);
            _robot_data->control.finger_width = r->value;
            _robot_data->control.gripper_control |= GRIPPER_COMMAND::TARGET_UPDATE_BIT;
            send_result_i32(fd, 0);
            break;
        }
        case OP_SET_TARGET_POSE: {
            if (plen != sizeof(ReqU8)) { send_err(fd, STATUS_INVALID_PAYLOAD); break; }
            auto* r = reinterpret_cast<ReqU8*>(payload.data());
            std::lock_guard<std::mutex> lk(_robot_data->mtx);
            _robot_data->control.finger_pose = r->value;
            _robot_data->control.gripper_control |= GRIPPER_COMMAND::TARGET_UPDATE_BIT;
            send_result_i32(fd, 0);
            break;
        }
        case OP_SET_TARGET_WIDTH_SPEED: {
            if (plen != sizeof(ReqU8)) { send_err(fd, STATUS_INVALID_PAYLOAD); break; }
            auto* r = reinterpret_cast<ReqU8*>(payload.data());
            std::lock_guard<std::mutex> lk(_robot_data->mtx);
            _robot_data->control.finger_width_speed = r->value;
            send_result_i32(fd, 0);
            break;
        }
        case OP_SET_TARGET_POSE_SPEED: {
            if (plen != sizeof(ReqU8)) { send_err(fd, STATUS_INVALID_PAYLOAD); break; }
            auto* r = reinterpret_cast<ReqU8*>(payload.data());
            std::lock_guard<std::mutex> lk(_robot_data->mtx);
            _robot_data->control.finger_pose_speed = r->value;
            send_result_i32(fd, 0);
            break;
        }
        case OP_SET_GRIPPING_FORCE: {
            if (plen != sizeof(ReqU8)) { send_err(fd, STATUS_INVALID_PAYLOAD); break; }
            auto* r = reinterpret_cast<ReqU8*>(payload.data());
            std::lock_guard<std::mutex> lk(_robot_data->mtx);
            _robot_data->control.gripping_force = r->value;
            send_result_i32(fd, 0);
            break;
        }
        case OP_SET_CONTACT_SENSITIVITY: {
            if (plen != sizeof(ReqU8)) { send_err(fd, STATUS_INVALID_PAYLOAD); break; }
            auto* r = reinterpret_cast<ReqU8*>(payload.data());
            std::lock_guard<std::mutex> lk(_robot_data->mtx);
            _robot_data->control.contact_sensitivity = r->value;
            send_result_i32(fd, 0);
            break;
        }
        case OP_RESET_ABS_ENCODER: {
            std::lock_guard<std::mutex> lk(_robot_data->mtx);
            _robot_data->control.gripper_control |= GRIPPER_COMMAND::ABS_ENCODER_RESET_BIT;
            _robot_data->resetAbsEncoder = true;
            send_result_i32(fd, 0);
            break;
        }
        case OP_RESET_FRICTION_MODEL: {
            std::lock_guard<std::mutex> lk(_robot_data->mtx);
            _robot_data->control.gripper_control |= GRIPPER_COMMAND::FRICTION_MODEL_ID_BIT;
            _robot_data->resetFriction = true;
            send_result_i32(fd, 0);
            break;
        }
        case OP_SHUTDOWN: {
            send_result_i32(fd, 0);
            _exit_flag = true;
            return;
        }
        default:
            send_err(fd, STATUS_INVALID_OP);
            break;
        }
    }
}
