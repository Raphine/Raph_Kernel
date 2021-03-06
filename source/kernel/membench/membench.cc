/*
 *
 * Copyright (c) 2017 Raphine Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Author: Liva
 * 
 */

#include <raph.h>
#include <task.h>
#include <cpu.h>
#include "cache.h"
#include <global.h>
#include "sync.h"
#include "membench.h"

SyncLow sync_1={0};
Sync2Low sync_2={0};
Sync2Low sync_3={0};
SyncLow sync_4={0};

Uint64 f_array[256];
volatile Uint64 monitor[37 * 8];


#define bench_func(x) membench8(x)

void bench_func(sptr<TaskWithStack> task);

CacheCtrl *cache_ctrl;

void beep(int argc, const char *argv[]);

void register_membench2_callout() {
  for (int i = 0; i < cpu_ctrl->GetHowManyCpus(); i++) {
    CpuId cpuid(i);
    
    auto task_ = make_sptr(new TaskWithStack(cpuid));
    task_->Init();
    task_->SetFunc(make_uptr(new Function<sptr<TaskWithStack>>([](sptr<TaskWithStack> task){
            int raw_cpuid = cpu_ctrl->GetCpuId().GetRawId();
            if (raw_cpuid == 0) {
              cache_ctrl = new CacheCtrl;
              cache_ctrl->Init();
            }
            bench_func(task);
            if (raw_cpuid == 0) {
              beep(0, nullptr);
            }
          }, task_)));
    task_ctrl->Register(cpuid, task_);
  }
}

