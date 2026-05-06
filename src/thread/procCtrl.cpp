#include "thread/procCtrl.h"
#include "thread/dtThread.h"
#include "device/deviceMan.h"
#include "util/dtTerm.h"
#include "sysData.h"

void *ThreadProcCtrl(void *arg)
{
    SysData *sysData = (SysData *)arg;
    RobotData *robotData = sysData->robotData;

    int Cnt = 0;
    timespec sleepTime;

    sysData->ctrlTime.targetPeriod_ms = 10.0; 
    sysData->dispTime.targetPeriod_ms = 10.0; 

    while (sysData->threadStart != 1)
    {
        dt::Thread::SleepForMillis(1);
    }
    dt::Thread::PostAllSemaphore();
    clock_gettime(CLOCK_MONOTONIC, &sleepTime);
    
    while (sysData->run)
    {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sleepTime, NULL);

        sleepTime.tv_nsec += sysData->ctrlTime.targetPeriod_ms * 1E6;
        if (sleepTime.tv_nsec >= 1E9)
        {
            sleepTime.tv_sec++;
            sleepTime.tv_nsec -= 1E9;
        }

        readDevice(robotData);

        ///////////////////////////////
        /* User Computation Function */

        // Initialization
        if (robotData->initProcess == 0)
        {
            // Gripper start command
            robotData->control.gripper_control = GRIPPER_COMMAND::SERVO_SWITCH_BIT;
            robotData->initProcess++;
        }
        else if (robotData->initProcess == 1)
        {
            if (robotData->status.gripper_status & GRIPPER_STATUS::READY_BIT)
            {
                dtTerm::Print("Gripper started.\n");
                robotData->control.finger_width = robotData->status.finger_width;
                robotData->control.finger_pose = robotData->status.finger_pose;
                robotData->control.finger_width_speed = 150;
                robotData->control.finger_pose_speed = 180;
                robotData->control.gripping_force = 30;
                robotData->control.contact_sensitivity = 80;
                robotData->initProcess++;
            }
        }

        // Clear gripper target update bit
        if (robotData->control.gripper_control & GRIPPER_COMMAND::TARGET_UPDATE_BIT)
        {
            if (robotData->status.gripper_status & GRIPPER_STATUS::IN_MOTION_BIT)
            {
                robotData->control.gripper_control &= ~GRIPPER_COMMAND::TARGET_UPDATE_BIT;
            }
        }

        // Clear absolute encoder reset bit
        if (robotData->resetAbsEncoder && ((robotData->status.gripper_status & GRIPPER_STATUS::READY_BIT) == 0))
        {
            robotData->control.gripper_control &= ~GRIPPER_COMMAND::ABS_ENCODER_RESET_BIT;
            robotData->resetAbsEncoder = false;
        }

        // Clear friction model reset bit
        if (robotData->resetFriction && ((robotData->status.gripper_status & GRIPPER_STATUS::READY_BIT) == 0))
        {
            robotData->control.gripper_control &= ~GRIPPER_COMMAND::FRICTION_MODEL_ID_BIT;
            robotData->resetFriction = false;
        }

        /* End of User Computation Function */
        //////////////////////////////////////

        writeDevice(robotData);
        Cnt++;

        // PostSemaphore(*sysData->dispSem);
        
    }
    return 0;
}