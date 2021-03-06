/*
 *
 * Copyright (c) 2016 Raphine Project
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
 * see IA-PC HPET Specification
 *
 */

#include "hpet.h"
#include <idt.h>
#include <global.h>

bool Hpet::SetupSub() {
  _dt = acpi_ctrl->GetHPETDT();
  if (_dt == nullptr) {
    return false;
  }
  phys_addr pbase = _dt->BaseAddr;
  _reg = reinterpret_cast<uint64_t *>(p2v(pbase));

  _cnt_clk_period = ((_reg[kRegGenCap] & kRegGenCapMaskMainCntPeriod) >> kRegGenCapOffsetMainCntPeriod) / (1000 * 1000);

  _table_num = ((_reg[kRegGenCap] & kRegGenCapMaskNumTimer)
                >> kRegGenCapOffsetNumTimer) + 1;

  if (_table_num == 0) {
    kernel_panic("hpet","not enough hpet timer");
  }
    
  // disable all timer
  for(int i = 0; i < _table_num; i++) {
    this->Disable(i);
  }

  // Enable Timer
  _reg[kRegGenConfig] |= kRegGenConfigFlagEnable;
  if ((_reg[kRegGenCap] & kRegGenCapLegacyRoute) != 0) {
    _reg[kRegGenConfig] |= kRegGenConfigFlagLegacy;
  }
  return true;
}

void Hpet::SetInt(CpuId cpuid, uint64_t cnt) {
  int id = 0;
  uint64_t config =
    kRegTmrConfigCapBaseFlagMode32 |
    kRegTmrConfigCapBaseFlagTypeNonPer;
  config &= ~kRegTmrConfigCapBaseMaskIntRoute;
  bool fsb_delivery = (_reg[GetRegTmrOfN(id, kBaseRegTmrConfigCap)] & kRegTmrConfigCapBaseFlagFsbIntDel) != 0;
  if (fsb_delivery) {
    config |=
      kRegTmrConfigCapBaseFlagFsbEnable |
      kRegTmrConfigCapBaseFlagIntTypeEdge;
    int vector = idt->SetIntCallback(cpuid, Handle, reinterpret_cast<void *>(this), Idt::EoiType::kLapic);
    _reg[kBaseRegFsbIntRoute] = (ApicCtrl::GetMsiAddr(apic_ctrl->GetApicIdFromCpuId(cpuid))<< 32) | ApicCtrl::GetMsiData(vector);
  } else {
    int pin = -1;
    if (_reg[kRegGenConfig] & kRegGenConfigFlagLegacy) {
      if (id == 0) {
        pin = 2;
      } else if (id == 1) {
        pin = 8;
      }
    }
    if (pin == -1) {
      uint32_t routecap = (_reg[GetRegTmrOfN(id, kBaseRegTmrConfigCap)] & kRegTmrConfigCapBaseMaskIntRouteCap) >> 32;
      for (int i = 0; i < 32; i++) {
        if ((routecap & (1 << i)) != 0) {
          pin = i;
          break;
        }
      }
    }
    if (pin == -1) {
      kernel_panic("hpet","unknown error");
    }
    config |=
      (pin << kRegTmrConfigCapBaseOffsetIntRoute) |
      kRegTmrConfigCapBaseFlagIntTypeLevel |
      kRegTmrConfigCapBaseFlagIntEnable;
    kassert(apic_ctrl != nullptr);
    int vector = idt->SetIntCallback(cpuid, Handle, reinterpret_cast<void *>(this), Idt::EoiType::kIoapic);
    kassert(apic_ctrl->SetupIoInt(pin, apic_ctrl->GetApicIdFromCpuId(cpuid), vector, false, true));
  }
  _reg[GetRegTmrOfN(id, kBaseRegTmrConfigCap)] = config;
  _reg[GetRegTmrOfN(id, kBaseRegTmrCmp)] = cnt;
}

void Hpet::Handle(Regs *rs, void *arg) {
  int id = 0;
  Hpet *that = reinterpret_cast<Hpet *>(arg);
  that->_reg[that->GetRegTmrOfN(id, kBaseRegTmrCmp)] = that->GetCntAfterPeriod(that->ReadMainCnt(), 1000 * 1000);
  bool fsb_delivery = (that->_reg[GetRegTmrOfN(id, kBaseRegTmrConfigCap)] & kRegTmrConfigCapBaseFlagFsbIntDel) != 0;
  if (!fsb_delivery) {
    that->_reg[0x20 / sizeof(uint64_t)] |= 1;
  }
}
