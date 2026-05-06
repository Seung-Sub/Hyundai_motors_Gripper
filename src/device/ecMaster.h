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
#ifndef MASTER_H
#define MASTER_H

#include <ecrt.h>
#include "sysData.h"

#define PERIOD_NS (1000000)
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)


int8_t ecMasterInit(SysData *sysData);
int8_t ecSetSlavePdo(uint16_t alias, uint16_t slavePos, uint32_t vid, uint32_t pid, const ec_sync_info_t syncs[]);
int8_t ecRegDomainPdoEntry(uint16_t index, uint8_t subindex, int *offset);
int8_t ecStart(SysData *sysData);
int8_t ecMasterM2S();
int8_t ecMasterS2M();

extern ec_master_t *master;
extern ec_domain_t *domain;

#endif