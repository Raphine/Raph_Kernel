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
 * 17/02/17: created by Levelfour
 * 
 */

#ifndef __RAPH_KERNEL_DEV_VOLTX_H__
#define __RAPH_KERNEL_DEV_VOLTX_H__

#include <stdint.h>
#include <mem/physmem.h>
#include <mem/virtmem.h>
#include <global.h>
#include <dev/eth.h>
#include <dev/pci.h>


struct VoltxRxDesc {
  uint64_t base_address;  /* address of buffer */
  uint16_t packet_length; /* length */
  uint8_t  reserved[6];
} __attribute__ ((packed));

struct VoltxTxDesc {
  uint64_t base_address;  /* address of buffer */
  uint16_t packet_length; /* length */
  uint8_t  reserved[6];
} __attribute__ ((packed));

class Voltx : public DevPci {
public:
  Voltx(uint8_t bus, uint8_t device, uint8_t function) : DevPci(bus, device, function), _voltx_eth(*this) {}
  static DevPci *InitPci(uint8_t bus, uint8_t device, uint8_t function);

  class VoltxEthernet : public DevEthernet {
  public:
    VoltxEthernet(Voltx &master) : _master(master) {
      _link_check_callout = make_sptr(new Callout());
      _link_check_callout->Init(make_uptr(new ClassFunction<VoltxEthernet, void *>(this, &VoltxEthernet::CheckLinkHandler, nullptr)));
    }

    void Setup();

    static void PollingHandler(Voltx *that);
    
    virtual void UpdateLinkStatus() override;
    
    // allocate 6 byte before call
    virtual void GetEthAddr(uint8_t *buffer) override;
    virtual void ChangeHandleMethodToPolling() override;
    virtual void ChangeHandleMethodToInt() override;
    virtual void Transmit(void *) override;
    Voltx &GetMasterClass() {
      return _master;
    }

    virtual bool IsLinkUp() override {
      return this->GetStatus() == NetDev::LinkStatus::kUp;
    }

  private:
    Voltx &_master;
    sptr<Callout> _link_check_callout;

    VoltxEthernet();
    void CheckLinkHandler(void *);

    SpinLock _lock;

    // Memory Mapped I/O Base Address
    volatile uint32_t *_mmioAddr = nullptr;

    // Controller Register Space
    static const uint32_t kRegCtrl   = 0x00;
    static const uint32_t kRegHadr0  = 0x04;
    static const uint32_t kRegHadr1  = 0x08;
    static const uint32_t kRegPadr0  = 0x0c;
    static const uint32_t kRegPadr1  = 0x10;
    static const uint32_t kRegPadr2  = 0x14;
    static const uint32_t kRegPadr3  = 0x18;
    static const uint32_t kRegTdba   = 0x1c;
    static const uint32_t kRegTdh    = 0x24;
    static const uint32_t kRegTdt    = 0x26;
    static const uint32_t kRegTdlen  = 0x28;
    static const uint32_t kRegRdba   = 0x2c;
    static const uint32_t kRegRdh    = 0x34;
    static const uint32_t kRegRdt    = 0x36;
    static const uint32_t kRegRdlen  = 0x38;

    // packet buffer
    static const uint32_t kTxdescNumber = 64;
    static const uint32_t kRxdescNumber = 64;
    VoltxTxDesc *_tx_desc_buf;
    VoltxRxDesc *_rx_desc_buf;

    static const uint32_t kMaxFrameLength = 1518;

    // MAC address
    uint8_t _ethAddr[6];

    void SetupRx();
    void SetupTx();

    int32_t Receive(uint8_t *buffer, uint32_t size);
    int32_t Transmit(const uint8_t *packet, uint32_t length);

    template<class T>
      void WriteMmio(uint32_t offset, T data) {
        *(reinterpret_cast<volatile T*>(_mmioAddr) + offset / sizeof(T)) = data;
      }

    template<class T>
      T ReadMmio(uint32_t offset) {
        return *(reinterpret_cast<volatile T*>(_mmioAddr) + offset / sizeof(T));
      }
  };

  VoltxEthernet &GetNetInterface() {
    return _voltx_eth;
  };

private:
  static const uint32_t kVendorId = 0x10ee; /* Xilinx */
  static const uint32_t kDeviceId = 0x6024; /* ML605 */

  VoltxEthernet _voltx_eth;
};

#endif /* __RAPH_KERNEL_DEV_VOLTX_H__ */
