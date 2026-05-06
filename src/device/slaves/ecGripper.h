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
#ifndef ECGRIPPER_H
#define ECGRIPPER_H

#include <ecrt.h>

#define VID_NETX 0xE0000044
#define PID_NETX 0x0000003D

#define GRIPPER_OBJ_GRIPPER_CONTROL 0x2000        // Gripper control (UNSIGNED8)
#define GRIPPER_OBJ_TARGET_WIDTH 0x2001           // Target width in mm (UNSIGNED8)
#define GRIPPER_OBJ_TARGET_POSE 0x2002            // Target pose in deg (UNSIGNED8)
#define GRIPPER_OBJ_SPEED_WIDTH 0x2003            // Gripping speed in mm/s (UNSIGNED8)
#define GRIPPER_OBJ_SPEED_POSE 0x2004             // Posing speed in deg/s (UNSIGNED8)
#define GRIPPER_OBJ_FORCE 0x2005                  // Gripping force in N (UNSIGNED8)
#define GRIPPER_OBJ_CONTACT_SENSITIVITY 0x2006    // Contact detection level (UNSIGNED8)
#define GRIPPER_OBJ_TARGET_CURRENT 0x2100         // Target current 1~4 (INTEGER16)
#define GRIPPER_OBJ_GRIPPER_STATUS 0x3000         // Gripper status (UNSIGNED8)
#define GRIPPER_OBJ_ACTUAL_WIDTH 0x3001           // Current width in mm (UNSIGNED8)
#define GRIPPER_OBJ_ACTUAL_POSE 0x3002            // Current pose in deg (UNSIGNED8)
#define GRIPPER_OBJ_STATUSWORD 0x3100             // Statusword 1~4 (UNSIGNED16)
#define GRIPPER_OBJ_POSITION_ACTUAL_VALUE 0x3101  // Position actual value 1~4 (INTEGER32)
#define GRIPPER_OBJ_AUXILIARY_ACTUAL_VALUE 0x3102 // Auxiliary actual value 1~4 (INTEGER32)
#define GRIPPER_OBJ_VELOCITY_ACTUAL_VALUE 0x3103  // Velocity actual value 1~4 (INTEGER32)
#define GRIPPER_OBJ_CURRENT_ACTUAL_VALUE 0x3104   // Current actual value 1~4 (INTEGER16)

#define GRIPPER_RPDO_0x1600_11 0x1600, 11
#define GRIPPER_RPDO_0x1600_ENTRY0 0x2000, 0x00, 8
#define GRIPPER_RPDO_0x1600_ENTRY1 0x2001, 0x00, 8
#define GRIPPER_RPDO_0x1600_ENTRY2 0x2002, 0x00, 8
#define GRIPPER_RPDO_0x1600_ENTRY3 0x2003, 0x00, 8
#define GRIPPER_RPDO_0x1600_ENTRY4 0x2004, 0x00, 8
#define GRIPPER_RPDO_0x1600_ENTRY5 0x2005, 0x00, 8
#define GRIPPER_RPDO_0x1600_ENTRY6 0x2006, 0x00, 8
#define GRIPPER_RPDO_0x1600_ENTRY7 0x2100, 0x01, 16
#define GRIPPER_RPDO_0x1600_ENTRY8 0x2100, 0x02, 16
#define GRIPPER_RPDO_0x1600_ENTRY9 0x2100, 0x03, 16
#define GRIPPER_RPDO_0x1600_ENTRY10 0x2100, 0x04, 16

#define GRIPPER_TPDO_0x1A00_23 0x1A00, 23
#define GRIPPER_TPDO_0x1A00_ENTRY0 0x3000, 0x00, 8
#define GRIPPER_TPDO_0x1A00_ENTRY1 0x3001, 0x00, 8
#define GRIPPER_TPDO_0x1A00_ENTRY2 0x3002, 0x00, 8
#define GRIPPER_TPDO_0x1A00_ENTRY3 0x3100, 0x01, 16
#define GRIPPER_TPDO_0x1A00_ENTRY4 0x3100, 0x02, 16
#define GRIPPER_TPDO_0x1A00_ENTRY5 0x3100, 0x03, 16
#define GRIPPER_TPDO_0x1A00_ENTRY6 0x3100, 0x04, 16
#define GRIPPER_TPDO_0x1A00_ENTRY7 0x3101, 0x01, 32
#define GRIPPER_TPDO_0x1A00_ENTRY8 0x3101, 0x02, 32
#define GRIPPER_TPDO_0x1A00_ENTRY9 0x3101, 0x03, 32
#define GRIPPER_TPDO_0x1A00_ENTRY10 0x3101, 0x04, 32
#define GRIPPER_TPDO_0x1A00_ENTRY11 0x3102, 0x01, 32
#define GRIPPER_TPDO_0x1A00_ENTRY12 0x3102, 0x02, 32
#define GRIPPER_TPDO_0x1A00_ENTRY13 0x3102, 0x03, 32
#define GRIPPER_TPDO_0x1A00_ENTRY14 0x3102, 0x04, 32
#define GRIPPER_TPDO_0x1A00_ENTRY15 0x3103, 0x01, 32
#define GRIPPER_TPDO_0x1A00_ENTRY16 0x3103, 0x02, 32
#define GRIPPER_TPDO_0x1A00_ENTRY17 0x3103, 0x03, 32
#define GRIPPER_TPDO_0x1A00_ENTRY18 0x3103, 0x04, 32
#define GRIPPER_TPDO_0x1A00_ENTRY19 0x3104, 0x01, 16
#define GRIPPER_TPDO_0x1A00_ENTRY20 0x3104, 0x02, 16
#define GRIPPER_TPDO_0x1A00_ENTRY21 0x3104, 0x03, 16
#define GRIPPER_TPDO_0x1A00_ENTRY22 0x3104, 0x04, 16

inline const ec_pdo_entry_info_t gripper_rpdo_entries[] = {
    {GRIPPER_RPDO_0x1600_ENTRY0},
    {GRIPPER_RPDO_0x1600_ENTRY1},
    {GRIPPER_RPDO_0x1600_ENTRY2},
    {GRIPPER_RPDO_0x1600_ENTRY3},
    {GRIPPER_RPDO_0x1600_ENTRY4},
    {GRIPPER_RPDO_0x1600_ENTRY5},
    {GRIPPER_RPDO_0x1600_ENTRY6},
    {GRIPPER_RPDO_0x1600_ENTRY7},
    {GRIPPER_RPDO_0x1600_ENTRY8},
    {GRIPPER_RPDO_0x1600_ENTRY9},
    {GRIPPER_RPDO_0x1600_ENTRY10}};
inline const ec_pdo_entry_info_t gripper_tpdo_entries[] = {
    {GRIPPER_TPDO_0x1A00_ENTRY0},
    {GRIPPER_TPDO_0x1A00_ENTRY1},
    {GRIPPER_TPDO_0x1A00_ENTRY2},
    {GRIPPER_TPDO_0x1A00_ENTRY3},
    {GRIPPER_TPDO_0x1A00_ENTRY4},
    {GRIPPER_TPDO_0x1A00_ENTRY5},
    {GRIPPER_TPDO_0x1A00_ENTRY6},
    {GRIPPER_TPDO_0x1A00_ENTRY7},
    {GRIPPER_TPDO_0x1A00_ENTRY8},
    {GRIPPER_TPDO_0x1A00_ENTRY9},
    {GRIPPER_TPDO_0x1A00_ENTRY10},
    {GRIPPER_TPDO_0x1A00_ENTRY11},
    {GRIPPER_TPDO_0x1A00_ENTRY12},
    {GRIPPER_TPDO_0x1A00_ENTRY13},
    {GRIPPER_TPDO_0x1A00_ENTRY14},
    {GRIPPER_TPDO_0x1A00_ENTRY15},
    {GRIPPER_TPDO_0x1A00_ENTRY16},
    {GRIPPER_TPDO_0x1A00_ENTRY17},
    {GRIPPER_TPDO_0x1A00_ENTRY18},
    {GRIPPER_TPDO_0x1A00_ENTRY19},
    {GRIPPER_TPDO_0x1A00_ENTRY20},
    {GRIPPER_TPDO_0x1A00_ENTRY21},
    {GRIPPER_TPDO_0x1A00_ENTRY22}};

inline const ec_pdo_info_t ecGripper_rpdos[] = {
    {GRIPPER_RPDO_0x1600_11, gripper_rpdo_entries},
};

inline const ec_pdo_info_t ecGripper_tpdos[] = {
    {GRIPPER_TPDO_0x1A00_23, gripper_tpdo_entries},
};

inline const ec_sync_info_t ecGripper_syncs[] = {
    // index, dir, n_pdos, *pdos, watchdog_mode
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, ecGripper_rpdos, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, ecGripper_tpdos, EC_WD_DISABLE},
    {0xff}};

#endif