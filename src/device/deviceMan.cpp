#include "device/deviceMan.h"
#include "util/dtTerm.h"

static int offsetTargetWidth;
static int offsetTargetPose;
static int offsetSpeedWidth;
static int offsetSpeedPose;
static int offsetForce;
static int offsetGripperStatus;
static int offsetActualWidth;
static int offsetActualPose;
static int offsetGripperControl;
static int offsetContactSensitivity;

static uint8_t *addrTargetWidth;
static uint8_t *addrTargetPose;
static uint8_t *addrSpeedWidth;
static uint8_t *addrSpeedPose;
static uint8_t *addrForce;
static uint8_t *addrGripperStatus;
static uint8_t *addrActualWidth;
static uint8_t *addrActualPose;
static uint8_t *addrGripperControl;
static uint8_t *addrContactSensitivity;

int initDevice(SysData *sysData)
{
    ecMasterInit(sysData);

    uint8_t slavePos = 0;
    ecSetSlavePdo(0, slavePos, VID_NETX, PID_NETX, ecGripper_syncs);

    ecRegDomainPdoEntry(GRIPPER_OBJ_TARGET_WIDTH, 0x00, &offsetTargetWidth);
    ecRegDomainPdoEntry(GRIPPER_OBJ_TARGET_POSE, 0x00, &offsetTargetPose);
    ecRegDomainPdoEntry(GRIPPER_OBJ_SPEED_WIDTH, 0x00, &offsetSpeedWidth);
    ecRegDomainPdoEntry(GRIPPER_OBJ_SPEED_POSE, 0x00, &offsetSpeedPose);
    ecRegDomainPdoEntry(GRIPPER_OBJ_FORCE, 0x00, &offsetForce);
    ecRegDomainPdoEntry(GRIPPER_OBJ_GRIPPER_STATUS, 0x00, &offsetGripperStatus);
    ecRegDomainPdoEntry(GRIPPER_OBJ_ACTUAL_WIDTH, 0x00, &offsetActualWidth);
    ecRegDomainPdoEntry(GRIPPER_OBJ_ACTUAL_POSE, 0x00, &offsetActualPose);
    ecRegDomainPdoEntry(GRIPPER_OBJ_GRIPPER_CONTROL, 0x00, &offsetGripperControl);
    ecRegDomainPdoEntry(GRIPPER_OBJ_CONTACT_SENSITIVITY, 0x00, &offsetContactSensitivity);

    ecStart(sysData);

    addrTargetWidth = sysData->pPdoList + offsetTargetWidth;
    addrTargetPose = sysData->pPdoList + offsetTargetPose;
    addrSpeedWidth = sysData->pPdoList + offsetSpeedWidth;
    addrSpeedPose = sysData->pPdoList + offsetSpeedPose;
    addrForce = sysData->pPdoList + offsetForce;
    addrGripperStatus = sysData->pPdoList + offsetGripperStatus;
    addrActualWidth = sysData->pPdoList + offsetActualWidth;
    addrActualPose = sysData->pPdoList + offsetActualPose;
    addrGripperControl = sysData->pPdoList + offsetGripperControl;
    addrContactSensitivity = sysData->pPdoList + offsetContactSensitivity;

    return 0;
}

int writeDevice(RobotData *robotData)
{
    // Encode
    EC_WRITE_U8(addrTargetWidth, robotData->control.finger_width);
    EC_WRITE_U8(addrTargetPose, robotData->control.finger_pose);
    EC_WRITE_U8(addrSpeedWidth, robotData->control.finger_width_speed);
    EC_WRITE_U8(addrSpeedPose, robotData->control.finger_pose_speed);
    EC_WRITE_U8(addrForce, robotData->control.gripping_force);
    EC_WRITE_U8(addrGripperControl, robotData->control.gripper_control);
    EC_WRITE_U8(addrContactSensitivity, robotData->control.contact_sensitivity);
    ecMasterM2S();
    return 0;
}

int readDevice(RobotData *robotData)
{
    ecMasterS2M();
    robotData->status.finger_width = EC_READ_U8(addrActualWidth);
    robotData->status.finger_pose = EC_READ_U8(addrActualPose);
    robotData->status.gripper_status = EC_READ_U8(addrGripperStatus);
    return 0;
}

int closeDevice(SysData *sysData)
{
    // Disable gripper
    sysData->robotData->control.gripper_control = 0;
    writeDevice(sysData->robotData);

    // Exit Process
    return 0;
}

void CheckKeyboardInput(SysData *sysData)
{
    if (dtTerm::kbhit())
    {
        int keyInput = getchar();
        switch (keyInput)
        {
        case 'E':
        case 'e':
            sysData->run = false;
            break;
        case 'R':
        case 'r':
            sysData->dispRestart = 1;
            break;
        default:
            sysData->robotData->keyboardInput = keyInput;
            break;
        }
    }
}