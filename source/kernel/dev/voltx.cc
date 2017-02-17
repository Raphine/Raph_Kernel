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
 * Author: Levelfour
 * 
 */

#include <mem/paging.h>
#include <mem/physmem.h>
#include <mem/virtmem.h>
#include <dev/voltx.h>
#include <tty.h>
#include <global.h>

DevPci *Voltx::InitPci(uint8_t bus, uint8_t device, uint8_t function) {
  Voltx *voltx_inst = new Voltx(bus, device, function);
  uint16_t vid = pci_ctrl->ReadReg<uint16_t>(bus, device, function, PciCtrl::kVendorIDReg);
  uint16_t did = pci_ctrl->ReadReg<uint16_t>(bus, device, function, PciCtrl::kDeviceIDReg);

  if (vid == Voltx::kVendorId && did == Voltx::kDeviceId) {
    VoltxEthernet *voltx_eth = &voltx_inst->GetNetInterface();
    voltx_eth->Setup();
    netdev_ctrl->RegisterDevice(voltx_eth, "voltx");

    return voltx_inst;
  } else {
    // no device found
    delete voltx_inst;
    return nullptr;
  }
}

void Voltx::VoltxEthernet::GetEthAddr(uint8_t *buffer) {
  memcpy(buffer, _ethAddr, 6);
}

void Voltx::VoltxEthernet::ChangeHandleMethodToPolling() {
  _polling.Init(make_uptr(new Function<Voltx *>(PollingHandler, &GetMasterClass())));
  _polling.Register(cpu_ctrl->RetainCpuIdForPurpose(CpuPurpose::kHighPerformance));

  task_ctrl->RegisterCallout(_link_check_callout, cpu_ctrl->RetainCpuIdForPurpose(CpuPurpose::kLowPriority), 1000);
}

void Voltx::VoltxEthernet::ChangeHandleMethodToInt() {
  _polling.Remove();
}

void Voltx::VoltxEthernet::CheckLinkHandler(void *) {
  UpdateLinkStatus();
  
  task_ctrl->RegisterCallout(_link_check_callout, cpu_ctrl->RetainCpuIdForPurpose(CpuPurpose::kLowPriority), 1000);
}

void Voltx::VoltxEthernet::PollingHandler(Voltx *that) {
  VoltxEthernet net = that->GetNetInterface();
  uint8_t buf[kMaxFrameLength];
  int32_t len;

  if ((len = net.Receive(buf, kMaxFrameLength)) != -1) {
    Packet *packet;
    if (net._rx_reserved.Pop(packet)) {
      memcpy(packet->GetBuffer(), buf, len);
      packet->len = len;
      if (!net._rx_buffered.Push(packet)) {
        kassert(net._rx_reserved.Push(packet));
      }
    }
  }

  if (!net._tx_buffered.IsEmpty()) {
    Packet *packet;
    kassert(net._tx_buffered.Pop(packet));
    net.Transmit(packet->GetBuffer(), packet->len);
    net.ReuseTxBuffer(packet);
  }
}

void Voltx::VoltxEthernet::Setup() {
  // get PCI Base Address Registers
  phys_addr bar = _master.ReadReg<uint32_t>(PciCtrl::kBaseAddressReg0);
  kassert((bar & 0xf) == 0);
  phys_addr mmio_addr = bar & 0xfffffff0;
  _mmioAddr = reinterpret_cast<uint32_t*>(p2v(mmio_addr));

  // Enable BusMaster
  _master.WriteReg<uint16_t>(PciCtrl::kCommandReg, _master.ReadReg<uint16_t>(PciCtrl::kCommandReg) | PciCtrl::kCommandRegBusMasterEnableFlag | (1 << 10));

  // TODO: disable
  WriteMmio<uint32_t>(kRegCtrl, 0);

  // fetch MAC address
  uint32_t hadr0 = ReadMmio<uint32_t>(kRegHadr0);
  uint32_t hadr1 = ReadMmio<uint32_t>(kRegHadr1);
  _ethAddr[0] = (hadr0) & 0xff;
  _ethAddr[1] = (hadr0 >> 8) & 0xff;
  _ethAddr[2] = (hadr0 >> 16) & 0xff;
  _ethAddr[3] = (hadr0 >> 24) & 0xff;
  _ethAddr[4] = (hadr1) & 0xff;
  _ethAddr[5] = (hadr1 >> 8) & 0xff;

  gtty->Cprintf("[voltx] MAC address is %02x:%02x:%02x:%02x:%02x:%02x\n",
      _ethAddr[0],
      _ethAddr[1],
      _ethAddr[2],
      _ethAddr[3],
      _ethAddr[4],
      _ethAddr[5]);

  this->SetupTx();
  this->SetupRx();

  // N.B. IP address write address
  uint8_t ipaddr[4] = {192, 168, 3, 100};
  WriteMmio<uint32_t>(kRegPadr0, (ipaddr[0] << 24) | (ipaddr[1] << 16) | (ipaddr[2] << 8) | ipaddr[3]);
  gtty->Cprintf("[voltx] IP address is %d.%d.%d.%d\n",
      ipaddr[0],
      ipaddr[1],
      ipaddr[2],
      ipaddr[3]);

  uint32_t ipaddr2 = ReadMmio<uint32_t>(kRegPadr0);
  gtty->Cprintf("[voltx] ReadMmio(kRegPadr0) = %p\n", ipaddr2);

  // TODO: enable
  WriteMmio<uint32_t>(kRegCtrl, 0xffffffffu);

  timer->BusyUwait(5*1000*1000);
}

void Voltx::VoltxEthernet::SetupTx() {
  InitTxPacketBuffer();

  // set base address of ring buffer
  virt_addr tx_desc_buf_addr = ((virtmem_ctrl->Alloc(sizeof(VoltxTxDesc) * kTxdescNumber + 15) + 15) / 16) * 16;
  _tx_desc_buf = reinterpret_cast<VoltxTxDesc*>(tx_desc_buf_addr);

  // Warning: root complex discard 64bit-width data, so you must split
  //          64bit data to 32bit
  WriteMmio<uint32_t>(kRegTdba, k2p(tx_desc_buf_addr) & 0xffffffff);
  WriteMmio<uint32_t>(kRegTdba + 4, (k2p(tx_desc_buf_addr) >> 32));

  // N.B. debug
  gtty->Cprintf("[voltx] TDBA = 0x%p\n", reinterpret_cast<uint64_t>(k2p(tx_desc_buf_addr)));

  // set head and tail pointer of ring
  WriteMmio<uint16_t>(kRegTdh, 0);
  WriteMmio<uint16_t>(kRegTdt, 0);
  
  // set the size of the desc ring
  WriteMmio<uint16_t>(kRegTdlen, kTxdescNumber);

  // initialize rx desc ring buffer
  for(uint32_t i = 0; i < kTxdescNumber; i++) {
    VoltxTxDesc *txdesc = &_tx_desc_buf[i];
    txdesc->base_address = k2p(virtmem_ctrl->Alloc(kMaxFrameLength));
    txdesc->packet_length = 0;
  }
}

void Voltx::VoltxEthernet::SetupRx() {
  InitRxPacketBuffer();

  // set base address of ring buffer
  PhysAddr paddr;
  physmem_ctrl->Alloc(paddr, PagingCtrl::ConvertNumToPageSize(sizeof(VoltxRxDesc) * kRxdescNumber));
  phys_addr rx_desc_buf_paddr = paddr.GetAddr();
  virt_addr rx_desc_buf_vaddr = p2v(rx_desc_buf_paddr);
  _rx_desc_buf = reinterpret_cast<VoltxRxDesc*>(rx_desc_buf_vaddr);

  // Warning: root complex discard 64bit-width data, so you must split
  //          64bit data to 32bit
  WriteMmio<uint32_t>(kRegRdba, rx_desc_buf_paddr & 0xffffffff);
  WriteMmio<uint32_t>(kRegRdba + 4, rx_desc_buf_paddr >> 32);

  gtty->Cprintf("[voltx] RDBA = 0x%p\n", reinterpret_cast<uint64_t>(rx_desc_buf_paddr));

  // set head and tail pointer of ring
  WriteMmio<uint16_t>(kRegRdh, 0);
  WriteMmio<uint16_t>(kRegRdt, 0);

  // set the size of the desc ring
  WriteMmio<uint16_t>(kRegRdlen, kRxdescNumber);

  // initialize rx desc ring buffer
  for(uint32_t i = 0; i < kRxdescNumber; i++) {
    VoltxRxDesc *rxdesc = &_rx_desc_buf[i];
    rxdesc->base_address = k2p(virtmem_ctrl->Alloc(kMaxFrameLength));
    rxdesc->packet_length = 0;
    gtty->Cprintf("[voltx] rxdesc[%d].base_address = 0x%p; rxdesc = 0x%p\n",
        i, rxdesc->base_address, v2p(reinterpret_cast<virt_addr>(rxdesc)));
  }
}

void Voltx::VoltxEthernet::UpdateLinkStatus() {
  _status = NetDev::LinkStatus::kUp;
}

int32_t Voltx::VoltxEthernet::Receive(uint8_t *buffer, uint32_t size) {
  VoltxRxDesc *rxdesc;
  uint32_t rdh = ReadMmio<uint16_t>(kRegRdh);
  uint32_t rdt = ReadMmio<uint16_t>(kRegRdt);
  uint32_t length;
  int rx_available = (kRxdescNumber - rdt + rdh) % kRxdescNumber;

  if(rx_available > 0) {

    // if the packet is on the wire
    rxdesc = _rx_desc_buf + (rdt % kRxdescNumber);
    length = size < rxdesc->packet_length ? size : rxdesc->packet_length;
    memcpy(buffer, reinterpret_cast<uint8_t*>(p2v(rxdesc->base_address)), length);
    WriteMmio<uint16_t>(kRegRdt, (rdt + 1) % kRxdescNumber);

    // N.B. test
    gtty->Cprintf("[voltx] rx; length = %d;\n", rxdesc->packet_length);
//    gtty->Cprintf("x", buffer[0], "x", buffer[1], "s", " ",
//                 "x", buffer[2], "x", buffer[3], "s", " ",
//                 "x", buffer[4], "x", buffer[5], "s", " ",
//                 "x", buffer[6], "x", buffer[7], "s", " ",
//                 "x", buffer[8], "x", buffer[9], "s", " ",
//                 "x", buffer[10], "x", buffer[11], "s", " ",
//                 "x", buffer[12], "x", buffer[13], "s", " ",
//                 "x", buffer[14], "x", buffer[15], "s", " ",
//                 "x", buffer[16], "x", buffer[17], "s", " ",
//                 "x", buffer[18], "x", buffer[19], "s", "\n");
//    for(uint32_t i = 0; i < length; i++) {
//      if(buffer[i] < 0x10) gtty->Printf("d", 0);
//      gtty->Printf("x", buffer[i]);
//      if((i+1) % 16 == 0) gtty->Printf("s", "\n");
//      else if((i+1) % 16 == 8) gtty->Printf("s", ":");
//      else gtty->Printf("s", " ");
//    }
//    gtty->Printf("s", "\n");
    // ^ test end

    return length;
  } else {
    return -1;
  }
}

int32_t Voltx::VoltxEthernet::Transmit(const uint8_t *packet, uint32_t length) {
  VoltxTxDesc *txdesc;
  uint32_t tdh = ReadMmio<uint16_t>(kRegTdh);
  uint32_t tdt = ReadMmio<uint16_t>(kRegTdt);
  int tx_available = kTxdescNumber - ((kTxdescNumber - tdh + tdt) % kTxdescNumber);

  if(tx_available > 0) {
    // if _tx_desc_buf is not full
    gtty->Cprintf("Voltx::VoltxEthernet::Transmit %d %d\n", tdh, tdt);
    txdesc = _tx_desc_buf + (tdt % kTxdescNumber);
    memcpy(reinterpret_cast<uint8_t*>(p2v(txdesc->base_address)), packet, length);
    txdesc->packet_length = length;
    WriteMmio<uint16_t>(kRegTdh, (tdh + 1) % kTxdescNumber);
    return length;
  } else {
    return -1;
  }
}

void Voltx::VoltxEthernet::Transmit(void *) {
  if (!this->_tx_buffered.IsEmpty()) {
    Packet *packet;
    kassert(this->_tx_buffered.Pop(packet));
    this->Transmit(packet->GetBuffer(), packet->len);
    this->ReuseTxBuffer(packet);
  }
}
