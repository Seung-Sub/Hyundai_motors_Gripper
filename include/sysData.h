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
#ifndef SYSDATA_H_
#define SYSDATA_H_

//* C/C++ System Headers -----------------------------------------------------*/
#include <memory>

//* Other Lib Headers --------------------------------------------------------*/
#include <thread/dtThread.h>
#include <ecrt.h>
#include <robotData.h>
typedef struct _systemData
{
    bool run = true;
    int exitCode = 0;
    int threadStart = 0;            // thread hold and order
    int dispRestart = 0;            // terminal display restart flag

    unsigned int ecMasterIndex;
    ec_master_state_t *ecMasterState;
    ec_domain_state_t *ecDomainState;
    uint8_t *pPdoList;

    dt::Thread::ThreadTimeInfo ctrlTime;
    dt::Thread::ThreadTimeInfo dispTime;

    dt::Thread::SemInfo *dispSem;

    RobotData *robotData;

} SysData;

#endif