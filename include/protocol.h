// art_gripper_standalone protocol — binary TCP, length-prefixed messages.
// Designed for sub-millisecond IPC latency on loopback (mirror of Polymetis weight class).
//
// Wire format
//   Request : [u8 op | u32 payload_len | bytes payload]
//   Response: [u8 status | u32 payload_len | bytes payload]
// All multi-byte integers are little-endian. status=0 means OK.
#pragma once
#include <cstdint>

namespace art_gripper {

constexpr uint16_t DEFAULT_PORT = 50053; // mirror Polymetis port style (arm 50051, gripper 50052)
constexpr uint32_t MAX_PAYLOAD = 4096;
constexpr uint8_t  STATUS_OK   = 0;
constexpr uint8_t  STATUS_ERR  = 1;
constexpr uint8_t  STATUS_INVALID_OP = 2;
constexpr uint8_t  STATUS_INVALID_PAYLOAD = 3;

// Op codes — request side.
enum OpCode : uint8_t {
    OP_PING                   = 0x01,
    OP_GET_INFO               = 0x02,
    OP_GET_STATUS             = 0x03,
    OP_MOTOR_ON               = 0x04,
    OP_SET_TARGET             = 0x05,
    OP_SET_TARGET_WIDTH       = 0x06,
    OP_SET_TARGET_POSE        = 0x07,
    OP_SET_GRIPPING_FORCE     = 0x08,
    OP_SET_CONTACT_SENSITIVITY= 0x09,
    OP_RESET_ABS_ENCODER      = 0x0A,
    OP_RESET_FRICTION_MODEL   = 0x0B,
    OP_SET_TARGET_WIDTH_SPEED = 0x0C,
    OP_SET_TARGET_POSE_SPEED  = 0x0D,
    OP_SHUTDOWN               = 0xFF, // graceful daemon stop (admin)
};

#pragma pack(push, 1)

// Request payloads
struct ReqMotorOn       { uint8_t on; };               // 0=off, 1=on
struct ReqSetTarget {
    uint8_t finger_width;        // 0..100 mm
    uint8_t finger_pose;         // 0..180 deg (0=Grasp, 90=3-finger, 180=2-finger)
    uint8_t finger_width_speed;  // 1..200 mm/s
    uint8_t finger_pose_speed;   // 1..255 deg/s
    uint8_t gripping_force;      // 1..100 N
    uint8_t contact_sensitivity; // 1..100
};
struct ReqU8 { uint8_t value; };

// Response payloads
struct RespStatus { // OP_GET_STATUS — current gripper state.
    uint8_t  gripper_status;   // bitfield: bit0 READY, bit1 FAULT, bit2 IN_MOTION, bit3 CONTACT
    uint8_t  finger_width;     // current mm
    uint8_t  finger_pose;      // current deg
    uint8_t  _pad;             // alignment
    uint16_t status_word[4];   // per-motor status word
    int32_t  position[4];      // per-motor pos
    int32_t  velocity[4];      // per-motor vel
    int16_t  current[4];       // per-motor cur
    int16_t  _pad2;
};
struct RespResult { int32_t result; };
// OP_GET_INFO returns a length-prefixed UTF-8 string concatenation:
//   [u8 name_len][bytes name][u8 ver_len][bytes ver][u16 desc_len][bytes desc]

#pragma pack(pop)

} // namespace art_gripper
