#include "thread/procDisp.h"
#include "device/deviceMan.h"
#include "util/dtTerm.h"
#include "sysData.h"
#include "stdio.h"

static void PrintInit();
// static void PrintThreadState(int r, SysData *sysData);
static void PrintEtherCATState(int r, SysData *sysData);

void *ThreadProcDisp(void *arg)
{
    SysData *sysData = (SysData *)arg;

    WaitSemaphore(*sysData->dispSem);

    PrintInit();

    uint8_t tempCnt = 0;

    while (sysData->run)
    {
        WaitSemaphore(*sysData->dispSem);

        if (sysData->dispRestart)
        {
            PrintInit();
            sysData->dispRestart = 0;
        }

        if (tempCnt++ > 10)
        {

            PrintEtherCATState(6, sysData);


            // PrintThreadState(2, sysData);
            // PrintControlGain(6, sysData->robotData);
            // PrintControlState(10, sysData->robotData);
            // PrintRobotJointData(15, sysData->robotData);

            tempCnt = 0;
        }
        CheckKeyboardInput(sysData);

    }
    return 0;
}

static void PrintInit()
{
    dtTerm::ClearDisp();
    dtTerm::CurPos(1, 1);
    dtTerm::Printf(+1, 1, "***** Thread state *************************************************************");
    dtTerm::Printf(+2, 1, "ctrl period(ms) = 00.000, load(ms) = 00.000, maxload(ms) = 00.000               ");
    dtTerm::Printf(+3, 1, "disp period(ms) = 00.000, load(ms) = 00.000, maxload(ms) = 00.000               ");
    dtTerm::Printf(+4, 1, "                                                                                ");
    dtTerm::Printf(+5, 1, "***** EtherCAT Master State ****************************************************");
    dtTerm::Printf(+6, 1, "Link is xxxx          Slave: 00        AL state: xxxxxxxxxxxx                   ");
    dtTerm::Printf(+7, 1, "Working Counter: 00                    WC State: xxxxxxxxxxxx                   ");
    dtTerm::Printf(+8, 1, "                                                                                ");
    dtTerm::Printf(+9, 1, "***** Joint state **************************************************************");
    dtTerm::Printf(10, 1, "name[state]:  incPos   absPos      vel   desTor   actTor    volt  InvTmp   error");
    dtTerm::Printf(11, 1, "FLR[0xFFFF]:+0000.00 +0000.00 +0000.00 +0000.00 +0000.00 +000.00 +000.00  0x0000");


    dtTerm::Printf(12, 1, "***** IO Module state **********************************************************");
    dtTerm::Printf(13, 1, "name[state]:  incPos   absPos      vel   desTor   actTor    volt  InvTmp   error");
    dtTerm::Printf(14, 1, "FLR[0xFFFF]:+0000.00 +0000.00 +0000.00 +0000.00 +0000.00 +000.00 +000.00  0x0000");

    dtTerm::Flush();

}

static void PrintEtherCATState(int r, SysData *sysData)
{
    int rtn;
    char strEcLink[5];
    char strEcAlState[13];
    char strEcWcState[13];

    rtn = sysData->ecMasterState->link_up;
    if (rtn == 1)
        sprintf(strEcLink, "  up");
    else
        sprintf(strEcLink, "down");

    rtn = sysData->ecMasterState->al_states;
    if (rtn == 1)
        sprintf(strEcAlState, "INIT(0x01)  ");
    else if (rtn == 2)
        sprintf(strEcAlState, "PREOP(0x02) ");
    else if (rtn == 4)
        sprintf(strEcAlState, "SAFEOP(0x04)");
    else if (rtn == 8)
        sprintf(strEcAlState, "OP(0x08)    ");
    else
        sprintf(strEcAlState, "0x%02X        ", rtn);
    

    rtn = sysData->ecDomainState->wc_state;
    if (rtn == 0)
        sprintf(strEcWcState, "No exchanged");
    else if (rtn == 1)
        sprintf(strEcWcState, "Incomplete  ");
    else if (rtn == 2)
        sprintf(strEcWcState, "Complete    ");
    else
        sprintf(strEcWcState, "Unknown     ");

    dtTerm::Printf(r, 9, "%s", strEcLink);
    dtTerm::Printf(r, 30, "%02d",
                   sysData->ecMasterState->slaves_responding);
    dtTerm::Printf(r, 50, "%s", strEcAlState);
    r++;

    dtTerm::Printf(r, 18, "%02u",
                   sysData->ecDomainState->working_counter);
    dtTerm::Printf(r, 50, "%s", strEcWcState);

    dtTerm::Flush();

}