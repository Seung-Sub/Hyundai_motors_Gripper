/*
MIT License

Copyright (c) 2025 Hyundai Motor Company

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author: Sangyup Yi (sean.yi@hyundai.com)
*/
#ifndef ROBOTDATA_HH
#define ROBOTDATA_HH

#include <mutex>
#include <cstdint>
#include <array>

#define GRIPPER_CLOSE 0
#define GRIPPER_OPEN 100

typedef enum _GRIPPER_COMMAND
{
    SERVO_SWITCH_BIT = 0x01,      // bit 0
    TARGET_UPDATE_BIT = 0x02,     // bit 1
    RESERVED_1 = 0x04,            // bit 2
    RESERVED_2 = 0x08,            // bit 3
    RESERVED_3 = 0x10,            // bit 4
    FRICTION_MODEL_ID_BIT = 0x20, // bit 5
    ABS_ENCODER_RESET_BIT = 0x40, // bit 6
    COMM_MODE_BIT = 0x80,         // bit 7

} GRIPPER_COMMAND;

typedef enum _GRIPPER_STATUS
{
    READY_BIT = 0x01,
    FAULT_BIT = 0x02,
    IN_MOTION_BIT = 0x04,
    CONTACT_BIT = 0x08,

} GRIPPER_STATUS;

typedef struct _gripperStatus
{
public:
    uint8_t gripper_status = 0;
    uint8_t finger_width = 0;
    uint8_t finger_pose = 0;
    std::array<uint16_t, 4> status_word = {0, 0, 0, 0};
    std::array<int32_t, 4> position = {0, 0, 0, 0};
    std::array<int32_t, 4> position_auxiliary = {0, 0, 0, 0};
    std::array<int32_t, 4> velocity = {0, 0, 0, 0};
    std::array<int16_t, 4> current = {0, 0, 0, 0};
} GripperStatus;

typedef struct _gripperControl
{
public:
    uint8_t gripper_control = 0;
    uint8_t finger_width = 0;
    uint8_t finger_pose = 0;
    uint8_t finger_width_speed = 0;
    uint8_t finger_pose_speed = 0;
    uint8_t gripping_force = 0;
    uint8_t contact_sensitivity = 0;
    std::array<int16_t, 4> target_current = {0, 0, 0, 0};
} GripperControl;

typedef struct _robotData
{
    uint8_t initProcess = 0;
    uint16_t keyboardInput = 0;
    bool resetAbsEncoder = false;
    bool resetFriction = false;

    GripperStatus status;
    GripperControl control;
    std::mutex mtx;
} RobotData;

#endif // ROBOTDATA_HH