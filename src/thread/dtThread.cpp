#include "thread/dtThread.h"
#include "util/dtTerm.h"
#include <cstring> // strerror
#include <cerrno> // errno
#include <cstdio>

using namespace std;

namespace dt
{
namespace Thread
{

static int threadNum = 0; // thread list number
static int semNum = 0;
static dt_thread_t *threadList[32];
static dt_sem_t *semList[64];
static int maxCpuCnt = 0;

int PrintThreadAttr(const pthread_attr_t *attr)
{
    size_t memSize;
    struct sched_param schedparam;
    int rtnParam;

    if (pthread_attr_getstacksize(attr, &memSize)) goto error;
    dtTerm::Printf("  Stack size: %zd\n", memSize);

    if (pthread_attr_getguardsize(attr, &memSize)) goto error;
    dtTerm::Printf("  Guard size: %zd\n", memSize);

    if (pthread_attr_getschedpolicy(attr, &rtnParam)) goto error;
    if (rtnParam == SCHED_FIFO)
        dtTerm::Print("  Scheduling policy: SCHED_FIFO\n");
    else if (rtnParam == SCHED_RR)
        dtTerm::Print("  Scheduling policy: SCHED_RR\n");
    else if (rtnParam == SCHED_OTHER)
        dtTerm::Print("  Scheduling policy: SCHED_OTHER\n");
    else
        dtTerm::Print("  Scheduling policy: [unknown]\n");

    if (pthread_attr_getschedparam(attr, &schedparam)) goto error;
    dtTerm::Printf("  Scheduling priority: %d\n", schedparam.sched_priority);

    if (pthread_attr_getdetachstate(attr, &rtnParam)) goto error;
    if (rtnParam == PTHREAD_CREATE_DETACHED)
        dtTerm::Print("  Detach state: DETACHED\n");
    else if (rtnParam == PTHREAD_CREATE_JOINABLE)
        dtTerm::Print("  Detach state: JOINABLE\n");
    else
        dtTerm::Print("[unknown]\n");

    if (pthread_attr_getinheritsched(attr, &rtnParam)) goto error;
    if (rtnParam == PTHREAD_INHERIT_SCHED)
        dtTerm::Print("  Inherit scheduler: INHERIT\n");
    else if (rtnParam == PTHREAD_EXPLICIT_SCHED)
        dtTerm::Print("  Inherit scheduler: EXPLICIT\n");
    else
        dtTerm::Print("  Inherit scheduler: [unknown]\n");

    return 0;

error:
    fprintf(stderr, "!Error! PrintThreadAttr() : %s(%d)\n", strerror(errno), errno);
    dtTerm::PrintEndLine();
    return -1;
}



int GetCpuCount(void)
{
    // start by assuming a maximum of 128 hardware threads and keep growing until
    // the cpu_set_t is big enough to hold the mask for the entire machine
    for (int max_count = 128; true; max_count *= 2)
    {
        cpu_set_t *cpusetp = CPU_ALLOC(max_count);
        size_t setsize = CPU_ALLOC_SIZE(max_count);

        if (!sched_getaffinity(getpid(), setsize, cpusetp))
        {
            // success
            // maxCpuCnt = CPU_COUNT_S(setsize, cpusetp);
            maxCpuCnt = sysconf(_SC_NPROCESSORS_ONLN);
            dtTerm::Printf("Max CPU Core Count: %d\n", maxCpuCnt);
            CPU_FREE(cpusetp);
            break;
        }
        else if (errno != EINVAL)
        {
            // failure other than max_count being too small, just return 1
            maxCpuCnt = 1;
            CPU_FREE(cpusetp);
            break;
        }
    }

    return maxCpuCnt;
}



int CreateNonRtThread(ThreadInfo &thread)
{
    cpu_set_t cpuset;
    pthread_attr_t taskAttr;
    if (maxCpuCnt == 0) GetCpuCount();

    dtTerm::PrintTitle(" Create Thread ");
    dtTerm::Printf("Thread Name: %s\n", thread.name);

    /* Step 1. Check CPU assign */
    if (thread.cpuIdx < 0 || thread.cpuIdx >= maxCpuCnt)
    {
        dtTerm::Printf("CPU Index: %d ... user error: Check the CPU Index\n", thread.cpuIdx);
        thread.cpuIdx = 0;
        dtTerm::Printf("CPU Index: default(%d) ... ok\n", thread.cpuIdx);
    }
    else
        dtTerm::Printf("CPU Index: %d ... ok\n", thread.cpuIdx);
    CPU_ZERO(&cpuset);               // removes all CPUs from cpuset
    CPU_SET(thread.cpuIdx, &cpuset); // add CPU idx to the cpuset

    /* Step 2. Thread attribute setting */
    dtTerm::Printf("Set pthread attribute ... ");
    if (pthread_attr_init(&taskAttr)) goto error;
    if (pthread_attr_setinheritsched(&taskAttr, PTHREAD_EXPLICIT_SCHED)) goto error;
    if (pthread_attr_setschedpolicy(&taskAttr, SCHED_OTHER)) goto error;
    if (pthread_attr_setaffinity_np(&taskAttr, sizeof(cpuset), &cpuset)) goto error;
    if (pthread_attr_setdetachstate(&taskAttr, PTHREAD_CREATE_JOINABLE)) goto error;

    if (thread.stackSz > 0)
    {
        if (pthread_attr_setstacksize(&taskAttr, thread.stackSz)) goto error;
    }

    dtTerm::Print("ok\n");

    /* Step 3. Create Thread */
    dtTerm::Print("Create Non-RT Thread ... ");
    if (pthread_create(&thread.id, &taskAttr, thread.procFunc, thread.procFuncArg)) goto error;
    dtTerm::Print("ok\n");

    /* Step 4. Check and Destroy the Attribute */
    PrintThreadAttr(&taskAttr);
    if (pthread_attr_destroy(&taskAttr)) goto error;

    dtTerm::Print("Complete\n");
    threadList[threadNum] = &thread.id;
    thread.listIdx = threadNum;
    threadNum++;
    dtTerm::PrintEndLine();

    return 0;

error:
    fprintf(stderr, "!Error! CreateNonRtThread() : %s(%d)\n", strerror(errno), errno);
    dtTerm::PrintEndLine();
    return -1;
}

int DeleteThread(ThreadInfo &thread)
{
    dtTerm::PrintTitle(" Delete Thread ");
    dtTerm::Printf("Delete %s ... ", thread.name);

    if (pthread_join(thread.id, NULL)) goto error;
    dtTerm::Print("ok\n");
    dtTerm::Print("Complete\n");

    threadList[thread.listIdx] = nullptr;
    dtTerm::PrintEndLine();

    return 0;

error:
    fprintf(stderr, "!Error! DeleteThread() : %s(%d)\n", strerror(errno), errno);
    dtTerm::PrintEndLine();
    return -1;
}

int DeleteAllThread()
{
    int num = 0;

    for (int idx = threadNum - 1; idx >= 0; idx--)
    {
        if (threadList[idx] == nullptr) continue;
        if (pthread_join(*threadList[idx], NULL)) goto error;
        num++;
    }
    dtTerm::Printf(43, 1, "--------------------------------");
    dtTerm::PrintTitle(" Delete All Thread ");
    dtTerm::Printf("Delete %d thread ... ok\n", num);
    dtTerm::Print("Complete\n");
    dtTerm::PrintEndLine();

    return 0;

error:
    dtTerm::PrintTitle(" Delete All Thread ");
    fprintf(stderr, "!Error! DeleteAllThread() : %s(%d)\n", strerror(errno), errno);
    dtTerm::PrintEndLine();
    return -1;
}


int CreateSemaphore(SemInfo &semInfo, unsigned int initValue)
{
    dtTerm::PrintTitle(" Ctreate Semaphore ");
    dtTerm::Print("Initialize Semaphore ... ");

#if defined(__APPLE__)
    semInfo.sem = dispatch_semaphore_create(initValue);
#else
    if (sem_init(&semInfo.sem, 0, initValue)) goto error; // 0 means semaphore may only be used by threads in the same process
#endif
    dtTerm::Print("ok\n");
    dtTerm::Print("Complete\n");
    semList[semNum] = &semInfo.sem;
    semInfo.listIdx = semNum;
    semNum++;
    dtTerm::PrintEndLine();

    return 0;

error:
    fprintf(stderr, "!Error! CreateSemaphore() : %s(%d)\n", strerror(errno), errno);
    dtTerm::PrintEndLine();
    return -1;
}

void PostAllSemaphore()
{
    for (int idx = semNum - 1; idx >= 0; idx--)
    {
        if (semList[idx] == nullptr) continue;
#if defined(__APPLE__)
        dispatch_semaphore_signal(*semList[idx]);
#else
        sem_post(semList[idx]);
#endif
    }
}

int DeleteSemaphore(SemInfo &semInfo)
{
    dtTerm::PrintTitle(" Delete Semaphore ");
    dtTerm::Print("Destroy Semaphore ... ");
#if defined(__APPLE__)
    dispatch_release(semInfo.sem);
#else
    if (sem_destroy(&semInfo.sem)) goto error;
#endif
    dtTerm::Print("ok\n");
    dtTerm::Print("Complete\n");
    semList[semInfo.listIdx] = nullptr;
    dtTerm::PrintEndLine();

    return 0;

error:
    fprintf(stderr, "!Error! DeleteSemaphore() : %s(%d)\n", strerror(errno), errno);
    dtTerm::PrintEndLine();
    return -1;
}

int DeleteAllSemaphore()
{
    int num = 0;

    dtTerm::PrintTitle(" Delete All Semaphore ");
    for (int idx = semNum - 1; idx >= 0; idx--)
    {
        if (semList[idx] == nullptr) continue;
#if defined(__APPLE__)
        dispatch_release(*semList[idx]);
#else
        if (sem_destroy(semList[idx])) goto error;
#endif
        num++;
    }
    dtTerm::Printf("Delete %d semaphore ... ok\n", num);
    dtTerm::Print("Complete\n");
    dtTerm::PrintEndLine();

    return 0;

error:
    fprintf(stderr, "!Error! DeleteAllSemaphore() : %s(%d)\n", strerror(errno), errno);
    dtTerm::PrintEndLine();
    return -1;
}
}}