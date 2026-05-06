#include "thread/threadMan.h"

#include "thread/dtThread.h"
#include "thread/procCtrl.h"
#include "thread/procDisp.h"

static dt::Thread::ThreadInfo ctrlThread = {"ControlThread", &ThreadProcCtrl};
// static dt::Thread::ThreadInfo dispThread = {"DisplayThread", &ThreadProcDisp};

static dt::Thread::SemInfo dispSem;

int initThread(SysData *sysData)
{
    // sysData->dispSem = &dispSem;
    // if (dt::Thread::CreateSemaphore(dispSem, 0)) return -1;
    
    ctrlThread.procFuncArg = sysData;
    // dispThread.procFuncArg = sysData;

    if (dt::Thread::CreateNonRtThread(ctrlThread)) return -1;
    // if (dt::Thread::CreateNonRtThread(dispThread)) return -1;

    sysData->threadStart = 1;

    return 0;
}

int closeThread()
{
    dt::Thread::PostAllSemaphore();
    if (dt::Thread::DeleteAllThread()) return -1;
    if (dt::Thread::DeleteAllSemaphore()) return -1;
    return 0;
}