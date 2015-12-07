/*
 *
 * Copyright (c) 2015 Raphine Project
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

#ifndef __RAPH_KERNEL_APIC_H__
#define __RAPH_KERNEL_APIC_H__

#include <stdint.h>
#include "acpi.h"

struct ACPISDTHeader;
struct MADTSt;

// see acpi spec
struct MADTStLAPIC {
  MADTSt st;
  uint8_t pid;
  uint8_t apicId;
  uint32_t flags;
} __attribute__ ((packed));

// see acpi spec
struct MADTStIOAPIC {
  MADTSt st;
  uint8_t length;
  uint8_t ioapicId;
  uint8_t reserved;
  uint32_t ioapicAddr;
  uint32_t glblIntBase;
} __attribute__ ((packed));

class ApicCtrl {
public:
  ApicCtrl() {
  }
  void Setup();
  void SetMADT(MADT *table) {
    _madt = table;
  }
private:
  MADT *_madt = nullptr;
  class Lapic {
  public:
    void Setup();
    void SetCtrlAddr(uint32_t *ctrlAddr) {
      _ctrlAddr = ctrlAddr;
    }
  private:
    uint32_t *_ctrlAddr = nullptr;
    static const int kIa32ApicBaseMsr = 0x1B;
    static const uint32_t kApicGlobalEnableFlag = 1 << 11;
    // see intel64 manual vol3 Table 10-1 (Local APIC Register Address Map)
    static const int kRegSvr = 0xF0 / sizeof(uint32_t);
    static const int kRegLvtTimer = 0x320 / sizeof(uint32_t);
    static const int kRegLvtThermalSensor = 0x330 / sizeof(uint32_t);
    static const int kRegLvtPerformanceCnt = 0x340 / sizeof(uint32_t);
    static const int kRegLvtLint0 = 0x350 / sizeof(uint32_t);
    static const int kRegLvtLint1 = 0x360 / sizeof(uint32_t);
    static const int kRegLvtErr = 0x370 / sizeof(uint32_t);

    // see intel64 manual vol3 Figure 10-8 (Local Vector Table)
    static const int kRegLintMask = 1 << 16;

    // see intel64 manual vol3 Figure 10-23 (Spurious-Interrupt Vector Register)
    static const uint32_t kRegSvrApicEnableFlag = 1 << 8;
  } _lapic;
  class Ioapic {
  public:
    void Setup();
    uint32_t Read(uint32_t index) {
      _reg[kIndex] = index;
      return _reg[kData];
    }
    void Write(uint32_t index, uint32_t data) {
      _reg[kIndex] = index;
      _reg[kData] = data;
    }
    void SetReg(uint32_t *reg) {
      _reg = reg;
    }
  private:
    uint32_t *_reg = nullptr;
    static const int kIndex = 0x0;
    static const int kData = 0x10 / sizeof(uint32_t);
  } _ioapic;
};

#endif /* __RAPH_KERNEL_APIC_H__ */
