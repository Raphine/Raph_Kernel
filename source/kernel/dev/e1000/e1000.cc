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
 */

#include <stdint.h>
#include <string.h>
#include "e1000.h"
#include "../../timer.h"
#include "../../mem/paging.h"
#include "../../mem/virtmem.h"
#include "../../mem/physmem.h"
#include "../../tty.h"
#include "../../global.h"

#include "e1000_raph.h"
#include "e1000_api.h"
#include "e1000_osdep.h"
#include "if_lem.h"

/*********************************************************************
 *  Legacy Em Driver version:
 *********************************************************************/
char lem_driver_version[] = "1.1.0";

/*********************************************************************
 *  PCI Device ID Table
 *
 *  Used by probe to select devices to load on
 *  Last field stores an index into e1000_strings
 *  Last entry must be all 0s
 *
 *  { Vendor ID, Device ID, SubVendor ID, SubDevice ID, String Index }
 *********************************************************************/

static em_vendor_info_t lem_vendor_info_array[] =
  {
    /* Intel(R) PRO/1000 Network Connection */
    { 0x8086, E1000_DEV_ID_82540EM,		PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82540EM_LOM,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82540EP,		PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82540EP_LOM,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82540EP_LP,	PCI_ANY_ID, PCI_ANY_ID, 0},

    { 0x8086, E1000_DEV_ID_82541EI,		PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82541ER,		PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82541ER_LOM,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82541EI_MOBILE,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82541GI,		PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82541GI_LF,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82541GI_MOBILE,	PCI_ANY_ID, PCI_ANY_ID, 0},

    { 0x8086, E1000_DEV_ID_82542,		PCI_ANY_ID, PCI_ANY_ID, 0},

    { 0x8086, E1000_DEV_ID_82543GC_FIBER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82543GC_COPPER,	PCI_ANY_ID, PCI_ANY_ID, 0},

    { 0x8086, E1000_DEV_ID_82544EI_COPPER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82544EI_FIBER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82544GC_COPPER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82544GC_LOM,	PCI_ANY_ID, PCI_ANY_ID, 0},

    { 0x8086, E1000_DEV_ID_82545EM_COPPER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82545EM_FIBER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82545GM_COPPER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82545GM_FIBER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82545GM_SERDES,	PCI_ANY_ID, PCI_ANY_ID, 0},

    { 0x8086, E1000_DEV_ID_82546EB_COPPER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82546EB_FIBER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82546EB_QUAD_COPPER, PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82546GB_COPPER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82546GB_FIBER,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82546GB_SERDES,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82546GB_PCIE,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82546GB_QUAD_COPPER, PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3,
      PCI_ANY_ID, PCI_ANY_ID, 0},

    { 0x8086, E1000_DEV_ID_82547EI,		PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82547EI_MOBILE,	PCI_ANY_ID, PCI_ANY_ID, 0},
    { 0x8086, E1000_DEV_ID_82547GI,		PCI_ANY_ID, PCI_ANY_ID, 0},
    /* required last entry */
    { 0, 0, 0, 0, 0}
  };

/*********************************************************************
 *  Table of branding strings for all supported NICs.
 *********************************************************************/

static const char *lem_strings[] = {
  "Intel(R) PRO/1000 Legacy Network Connection"
};

/*********************************************************************
 *  Function prototypes
 *********************************************************************/
static int	lem_probe(device_t);
static int	lem_attach(device_t);
// static int	lem_detach(device_t);
// static int	lem_shutdown(device_t);
// static int	lem_suspend(device_t);
// static int	lem_resume(device_t);
static void	lem_start(if_t);
static void	lem_start_locked(if_t ifp);
// static int	lem_ioctl(if_t, u_long, caddr_t);
// #if __FreeBSD_version >= 1100036
// static uint64_t	lem_get_counter(if_t, ift_counter);
// #endif
static void	lem_init(struct adapter *);
static void	lem_init_locked(struct adapter *);
static void	lem_stop(struct adapter *);
// static void	lem_media_status(if_t, struct ifmediareq *);
// static int	lem_media_change(if_t);
static void	lem_identify_hardware(struct adapter *);
static int	lem_allocate_pci_resources(struct adapter *);
static int	lem_allocate_irq(struct adapter *adapter);
// static void	lem_free_pci_resources(struct adapter *);
static void	lem_local_timer(void *);
static int	lem_hardware_init(struct adapter *);
static int	lem_setup_interface(device_t, struct adapter *);
static void	lem_setup_transmit_structures(struct adapter *);
static void	lem_initialize_transmit_unit(struct adapter *);
static int	lem_setup_receive_structures(struct adapter *);
static void	lem_initialize_receive_unit(struct adapter *);
// static void	lem_enable_intr(struct adapter *);
static void	lem_disable_intr(struct adapter *);
// static void	lem_free_transmit_structures(struct adapter *);
// static void	lem_free_receive_structures(struct adapter *);
static void	lem_update_stats_counters(struct adapter *);
// static void	lem_add_hw_stats(struct adapter *adapter);
static void	lem_txeof(struct adapter *);
// static void	lem_tx_purge(struct adapter *);
static int	lem_allocate_receive_structures(struct adapter *);
static int	lem_allocate_transmit_structures(struct adapter *);
static bool	lem_rxeof(struct adapter *, int, int *);
// #ifndef __NO_STRICT_ALIGNMENT
// static int	lem_fixup_rx(struct adapter *);
// #endif
// static void	lem_receive_checksum(struct adapter *, struct e1000_rx_desc *,
// 		    struct mbuf *);
// static void	lem_transmit_checksum_setup(struct adapter *, struct mbuf *,
// 		    u32 *, u32 *);
// static void	lem_set_promisc(struct adapter *);
// static void	lem_disable_promisc(struct adapter *);
static void	lem_set_multi(struct adapter *);
static void	lem_update_link_status(struct adapter *);
// static int	lem_get_buf(struct adapter *, int);
// static void	lem_register_vlan(void *, if_t, u16);
// static void	lem_unregister_vlan(void *, if_t, u16);
// static void	lem_setup_vlan_hw_support(struct adapter *);
static int	lem_xmit(struct adapter *, E1000::Packet *);
static void	lem_smartspeed(struct adapter *);
// static int	lem_82547_fifo_workaround(struct adapter *, int);
// static void	lem_82547_update_fifo_head(struct adapter *, int);
// static int	lem_82547_tx_fifo_reset(struct adapter *);
// static void	lem_82547_move_tail(void *);
static int	lem_dma_malloc(struct adapter *, bus_size_t,
                               struct em_dma_alloc *, int);
// static void	lem_dma_free(struct adapter *, struct em_dma_alloc *);
// static int	lem_sysctl_nvm_info(SYSCTL_HANDLER_ARGS);
// static void	lem_print_nvm_info(struct adapter *);
static int 	lem_is_valid_ether_addr(u8 *);
// static u32	lem_fill_descriptors (bus_addr_t address, u32 length,
// 		    PDESC_ARRAY desc_array);
// static int	lem_sysctl_int_delay(SYSCTL_HANDLER_ARGS);
// static void	lem_add_int_delay_sysctl(struct adapter *, const char *,
// 		    const char *, struct em_int_delay_info *, int, int);
// static void	lem_set_flow_cntrl(struct adapter *, const char *,
// 		    const char *, int *, int);
// /* Management and WOL Support */
static void	lem_init_manageability(struct adapter *);
// static void	lem_release_manageability(struct adapter *);
static void     lem_get_hw_control(struct adapter *);
// static void     lem_release_hw_control(struct adapter *);
static void	lem_get_wakeup(device_t);
// static void     lem_enable_wakeup(device_t);
// static int	lem_enable_phy_wakeup(struct adapter *);
// static void	lem_led_func(void *, int);

// static void	lem_intr(void *);
// static int	lem_irq_fast(void *);
// static void	lem_handle_rxtx(void *context, int pending);
// static void	lem_handle_link(void *context, int pending);
// static void	lem_add_rx_process_limit(struct adapter *, const char *,
// 		    const char *, int *, int);
static int lem_poll(if_t ifp);

/*********************************************************************
 *  Tunable default values.
 *********************************************************************/

#define EM_TICKS_TO_USECS(ticks)	((1024 * (ticks) + 500) / 1000)
#define EM_USECS_TO_TICKS(usecs)	((1000 * (usecs) + 512) / 1024)

#define MAX_INTS_PER_SEC	8000
#define DEFAULT_ITR		(1000000000/(MAX_INTS_PER_SEC * 256))

static int lem_tx_int_delay_dflt = EM_TICKS_TO_USECS(EM_TIDV);
static int lem_rx_int_delay_dflt = EM_TICKS_TO_USECS(EM_RDTR);
static int lem_tx_abs_int_delay_dflt = EM_TICKS_TO_USECS(EM_TADV);
static int lem_rx_abs_int_delay_dflt = EM_TICKS_TO_USECS(EM_RADV);
/*
 * increase lem_rxd and lem_txd to at least 2048 in netmap mode
 * for better performance.
 */
static int lem_rxd = EM_DEFAULT_RXD;
static int lem_txd = EM_DEFAULT_TXD;
static int lem_smart_pwr_down = FALSE;

/* Controls whether promiscuous also shows bad packets */
static int lem_debug_sbp = FALSE;

TUNABLE_INT("hw.em.tx_int_delay", &lem_tx_int_delay_dflt);
TUNABLE_INT("hw.em.rx_int_delay", &lem_rx_int_delay_dflt);
TUNABLE_INT("hw.em.tx_abs_int_delay", &lem_tx_abs_int_delay_dflt);
TUNABLE_INT("hw.em.rx_abs_int_delay", &lem_rx_abs_int_delay_dflt);
TUNABLE_INT("hw.em.rxd", &lem_rxd);
TUNABLE_INT("hw.em.txd", &lem_txd);
TUNABLE_INT("hw.em.smart_pwr_down", &lem_smart_pwr_down);
TUNABLE_INT("hw.em.sbp", &lem_debug_sbp);

/* Interrupt style - default to fast */
static int lem_use_legacy_irq = 0;
TUNABLE_INT("hw.em.use_legacy_irq", &lem_use_legacy_irq);

/* How many packets rxeof tries to clean at a time */
static int lem_rx_process_limit = 100;
TUNABLE_INT("hw.em.rx_process_limit", &lem_rx_process_limit);

/* Flow control setting - default to FULL */
static int lem_fc_setting = e1000_fc_full;
TUNABLE_INT("hw.em.fc_setting", &lem_fc_setting);

/* Global used in WOL setup with multiport cards */
static int global_quad_port_a = 0;

extern E1000 *eth;
void E1000::InitPCI(uint16_t vid, uint16_t did, uint8_t bus, uint8_t device, bool mf) {
  E1000 *addr = reinterpret_cast<E1000 *>(virtmem_ctrl->Alloc(sizeof(E1000)));
  addr = new(addr) E1000(bus, device, mf);
  addr->bsd.parent = addr;
  addr->bsd.adapter = reinterpret_cast<struct adapter *>(virtmem_ctrl->Alloc(sizeof(adapter)));
  new(&addr->bsd.adapter->timer.callout) Callout;
  new(&addr->bsd.adapter->tx_fifo_timer.callout) Callout;
  new(&addr->bsd.adapter->core_mtx.lock) SpinLock;
  new(&addr->bsd.adapter->tx_mtx.lock) SpinLock;
  new(&addr->bsd.adapter->rx_mtx.lock) SpinLock;
  if (lem_probe(&addr->bsd) == BUS_PROBE_DEFAULT) {
    kassert(lem_attach(&addr->bsd) == 0);
    lem_init(addr->bsd.adapter);
    polling_ctrl->Register(addr);
    eth = addr;
  } else {
    virtmem_ctrl->Free(ptr2virtaddr(addr->bsd.adapter));
    virtmem_ctrl->Free(ptr2virtaddr(addr));
  }  
}

void E1000::Handle() {
  lem_poll(this->bsd.adapter->ifp);
  lem_start(this->bsd.adapter->ifp);
}

void E1000::GetEthAddr(uint8_t *buffer) {
  memcpy(buffer, bsd.adapter->hw.mac.addr, 6);
}


static int
lem_probe(device_t dev)
{
  char		adapter_name[60];
  u16		pci_vendor_id = 0;
  u16		pci_device_id = 0;
  u16		pci_subvendor_id = 0;
  u16		pci_subdevice_id = 0;
  em_vendor_info_t *ent;

  INIT_DEBUGOUT("em_probe: begin");

  pci_vendor_id = pci_get_vendor(dev);
  if (pci_vendor_id != EM_VENDOR_ID)
    return (ENXIO);

  pci_device_id = pci_get_device(dev);
  pci_subvendor_id = pci_get_subvendor(dev);
  pci_subdevice_id = pci_get_subdevice(dev);

  ent = lem_vendor_info_array;
  while (ent->vendor_id != 0) {
    if ((pci_vendor_id == ent->vendor_id) &&
        (pci_device_id == ent->device_id) &&

        ((pci_subvendor_id == ent->subvendor_id) ||
         (ent->subvendor_id == PCI_ANY_ID)) &&

        ((pci_subdevice_id == ent->subdevice_id) ||
         (ent->subdevice_id == PCI_ANY_ID))) {
      sprintf(adapter_name, "%s %s",
              lem_strings[ent->index],
              lem_driver_version);
      device_set_desc_copy(dev, adapter_name);
      return (BUS_PROBE_DEFAULT);
    }
    ent++;
  }

  return (ENXIO);
}

/*********************************************************************
 *  Device initialization routine
 *
 *  The attach entry point is called when the driver is being loaded.
 *  This routine identifies the type of hardware, allocates all resources
 *  and initializes the hardware.
 *
 *  return 0 on success, positive on failure
 *********************************************************************/

static int
lem_attach(device_t dev)
{
  struct adapter	*adapter;
  int		tsize, rsize;
  int		error = 0;

  INIT_DEBUGOUT("lem_attach: begin");

  adapter = device_get_softc(dev);
  adapter->dev = adapter->osdep.dev = dev;
  EM_CORE_LOCK_INIT(adapter, device_get_nameunit(dev));
  EM_TX_LOCK_INIT(adapter, device_get_nameunit(dev));
  EM_RX_LOCK_INIT(adapter, device_get_nameunit(dev));

  /* SYSCTL stuff */
  SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
                  SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
                  OID_AUTO, "nvm", CTLTYPE_INT|CTLFLAG_RW, adapter, 0,
                  lem_sysctl_nvm_info, "I", "NVM Information");

  callout_init_mtx(&adapter->timer, &adapter->core_mtx, 0);
  callout_init_mtx(&adapter->tx_fifo_timer, &adapter->tx_mtx, 0);

  /* Determine hardware and mac info */
  lem_identify_hardware(adapter);

  /* Setup PCI resources */
  if (lem_allocate_pci_resources(adapter)) {
    device_printf(dev, "Allocation of PCI resources failed\n");
    error = ENXIO;
    goto err_pci;
  }

  /* Do Shared Code initialization */
  if (e1000_setup_init_funcs(&adapter->hw, TRUE)) {
    device_printf(dev, "Setup of Shared code failed\n");
    error = ENXIO;
    goto err_pci;
  }

  e1000_get_bus_info(&adapter->hw);

  /* Set up some sysctls for the tunable interrupt delays */
  // lem_add_int_delay_sysctl(adapter, "rx_int_delay",
  //                          "receive interrupt delay in usecs", &adapter->rx_int_delay,
  //                          E1000_REGISTER(&adapter->hw, E1000_RDTR), lem_rx_int_delay_dflt);
  // 	lem_add_int_delay_sysctl(adapter, "tx_int_delay",
  // 	    "transmit interrupt delay in usecs", &adapter->tx_int_delay,
  // 	    E1000_REGISTER(&adapter->hw, E1000_TIDV), lem_tx_int_delay_dflt);

  // 	if (adapter->hw.mac.type >= e1000_82540) {
  // 		lem_add_int_delay_sysctl(adapter, "rx_abs_int_delay",
  // 		    "receive interrupt delay limit in usecs",
  // 		    &adapter->rx_abs_int_delay,
  // 		    E1000_REGISTER(&adapter->hw, E1000_RADV),
  // 		    lem_rx_abs_int_delay_dflt);
  // 		lem_add_int_delay_sysctl(adapter, "tx_abs_int_delay",
  // 		    "transmit interrupt delay limit in usecs",
  // 		    &adapter->tx_abs_int_delay,
  // 		    E1000_REGISTER(&adapter->hw, E1000_TADV),
  // 		    lem_tx_abs_int_delay_dflt);
  // 		lem_add_int_delay_sysctl(adapter, "itr",
  // 		    "interrupt delay limit in usecs/4",
  // 		    &adapter->tx_itr,
  // 		    E1000_REGISTER(&adapter->hw, E1000_ITR),
  // 		    DEFAULT_ITR);
  // 	}

  /* Sysctls for limiting the amount of work done in the taskqueue */
  // 	lem_add_rx_process_limit(adapter, "rx_processing_limit",
  // 	    "max number of rx packets to process", &adapter->rx_process_limit,
  // 	    lem_rx_process_limit);

  // #ifdef NIC_SEND_COMBINING
  // 	/* Sysctls to control mitigation */
  // 	lem_add_rx_process_limit(adapter, "sc_enable",
  // 	    "driver TDT mitigation", &adapter->sc_enable, 0);
  // #endif /* NIC_SEND_COMBINING */
  // #ifdef BATCH_DISPATCH
  // 	lem_add_rx_process_limit(adapter, "batch_enable",
  // 	    "driver rx batch", &adapter->batch_enable, 0);
  // #endif /* BATCH_DISPATCH */
  // #ifdef NIC_PARAVIRT
  // 	lem_add_rx_process_limit(adapter, "rx_retries",
  // 	    "driver rx retries", &adapter->rx_retries, 0);
  // #endif /* NIC_PARAVIRT */

  //         /* Sysctl for setting the interface flow control */
  // 	lem_set_flow_cntrl(adapter, "flow_control",
  // 	    "flow control setting",
  // 	    &adapter->fc_setting, lem_fc_setting);

  /*
   * Validate number of transmit and receive descriptors. It
   * must not exceed hardware maximum, and must be multiple
   * of E1000_DBA_ALIGN.
   */
  if (((lem_txd * sizeof(struct e1000_tx_desc)) % EM_DBA_ALIGN) != 0 ||
      (adapter->hw.mac.type >= e1000_82544 && lem_txd > EM_MAX_TXD) ||
      (adapter->hw.mac.type < e1000_82544 && lem_txd > EM_MAX_TXD_82543) ||
      (lem_txd < EM_MIN_TXD)) {
    device_printf(dev, "Using %d TX descriptors instead of %d!\n",
                  EM_DEFAULT_TXD, lem_txd);
    adapter->num_tx_desc = EM_DEFAULT_TXD;
  } else
    adapter->num_tx_desc = lem_txd;
  if (((lem_rxd * sizeof(struct e1000_rx_desc)) % EM_DBA_ALIGN) != 0 ||
      (adapter->hw.mac.type >= e1000_82544 && lem_rxd > EM_MAX_RXD) ||
      (adapter->hw.mac.type < e1000_82544 && lem_rxd > EM_MAX_RXD_82543) ||
      (lem_rxd < EM_MIN_RXD)) {
    device_printf(dev, "Using %d RX descriptors instead of %d!\n",
                  EM_DEFAULT_RXD, lem_rxd);
    adapter->num_rx_desc = EM_DEFAULT_RXD;
  } else
    adapter->num_rx_desc = lem_rxd;

  adapter->hw.mac.autoneg = DO_AUTO_NEG;
  adapter->hw.phy.autoneg_wait_to_complete = FALSE;
  adapter->hw.phy.autoneg_advertised = AUTONEG_ADV_DEFAULT;
  adapter->rx_buffer_len = 2048;

  e1000_init_script_state_82541(&adapter->hw, TRUE);
  e1000_set_tbi_compatibility_82543(&adapter->hw, TRUE);

  /* Copper options */
  if (adapter->hw.phy.media_type == e1000_media_type_copper) {
    adapter->hw.phy.mdix = AUTO_ALL_MODES;
    adapter->hw.phy.disable_polarity_correction = FALSE;
    adapter->hw.phy.ms_type = EM_MASTER_SLAVE;
  }

  /*
   * Set the frame limits assuming
   * standard ethernet sized frames.
   */
  adapter->max_frame_size = ETHERMTU + ETHER_HDR_LEN + ETHERNET_FCS_SIZE;
  adapter->min_frame_size = ETH_ZLEN + ETHERNET_FCS_SIZE;

  /*
   * This controls when hardware reports transmit completion
   * status.
   */
  adapter->hw.mac.report_tx_early = 1;

  // #ifdef NIC_PARAVIRT
  // 	device_printf(dev, "driver supports paravirt, subdev 0x%x\n",
  // 		adapter->hw.subsystem_device_id);
  // 	if (adapter->hw.subsystem_device_id == E1000_PARA_SUBDEV) {
  // 		uint64_t bus_addr;

  // 		device_printf(dev, "paravirt support on dev %p\n", adapter);
  // 		tsize = 4096; // XXX one page for the csb
  // 		if (lem_dma_malloc(adapter, tsize, &adapter->csb_mem, BUS_DMA_NOWAIT)) {
  // 			device_printf(dev, "Unable to allocate csb memory\n");
  // 			error = ENOMEM;
  // 			goto err_csb;
  // 		}
  // 		/* Setup the Base of the CSB */
  // 		adapter->csb = (struct paravirt_csb *)adapter->csb_mem.dma_vaddr;
  // 		/* force the first kick */
  // 		adapter->csb->host_need_txkick = 1; /* txring empty */
  // 		adapter->csb->guest_need_rxkick = 1; /* no rx packets */
  // 		bus_addr = adapter->csb_mem.dma_paddr;
  // 		lem_add_rx_process_limit(adapter, "csb_on",
  // 		    "enable paravirt.", &adapter->csb->guest_csb_on, 0);
  // 		lem_add_rx_process_limit(adapter, "txc_lim",
  // 		    "txc_lim", &adapter->csb->host_txcycles_lim, 1);

  // 		/* some stats */
  // #define PA_SC(name, var, val)		\
  // 	lem_add_rx_process_limit(adapter, name, name, var, val)
  // 		PA_SC("host_need_txkick",&adapter->csb->host_need_txkick, 1);
  // 		PA_SC("host_rxkick_at",&adapter->csb->host_rxkick_at, ~0);
  // 		PA_SC("guest_need_txkick",&adapter->csb->guest_need_txkick, 0);
  // 		PA_SC("guest_need_rxkick",&adapter->csb->guest_need_rxkick, 1);
  // 		PA_SC("tdt_reg_count",&adapter->tdt_reg_count, 0);
  // 		PA_SC("tdt_csb_count",&adapter->tdt_csb_count, 0);
  // 		PA_SC("tdt_int_count",&adapter->tdt_int_count, 0);
  // 		PA_SC("guest_need_kick_count",&adapter->guest_need_kick_count, 0);
  // 		/* tell the host where the block is */
  // 		E1000_WRITE_REG(&adapter->hw, E1000_CSBAH,
  // 			(u32)(bus_addr >> 32));
  // 		E1000_WRITE_REG(&adapter->hw, E1000_CSBAL,
  // 			(u32)bus_addr);
  // 	}
  // #endif /* NIC_PARAVIRT */

  tsize = roundup2(adapter->num_tx_desc * sizeof(struct e1000_tx_desc),
                   EM_DBA_ALIGN);

  /* Allocate Transmit Descriptor ring */
  if (lem_dma_malloc(adapter, tsize, &adapter->txdma, BUS_DMA_NOWAIT)) {
    device_printf(dev, "Unable to allocate tx_desc memory\n");
    error = ENOMEM;
    goto err_tx_desc;
  }
  adapter->tx_desc_base = 
    (struct e1000_tx_desc *)adapter->txdma.dma_vaddr;

  rsize = roundup2(adapter->num_rx_desc * sizeof(struct e1000_rx_desc),
                   EM_DBA_ALIGN);

  /* Allocate Receive Descriptor ring */
  if (lem_dma_malloc(adapter, rsize, &adapter->rxdma, BUS_DMA_NOWAIT)) {
    device_printf(dev, "Unable to allocate rx_desc memory\n");
    error = ENOMEM;
    goto err_rx_desc;
  }
  adapter->rx_desc_base =
    (struct e1000_rx_desc *)adapter->rxdma.dma_vaddr;

  /* Allocate multicast array memory. */
  adapter->mta = reinterpret_cast<u8 *>(virtmem_ctrl->Alloc(sizeof(u8) * ETH_ADDR_LEN *
                                                            MAX_NUM_MULTICAST_ADDRESSES));
  if (adapter->mta == NULL) {
    device_printf(dev, "Can not allocate multicast setup array\n");
    error = ENOMEM;
    goto err_hw_init;
  }

  /*
  ** Start from a known state, this is
  ** important in reading the nvm and
  ** mac from that.
  */
  e1000_reset_hw(&adapter->hw);

  /* Make sure we have a good EEPROM before we read from it */
  if (e1000_validate_nvm_checksum(&adapter->hw) < 0) {
    /*
    ** Some PCI-E parts fail the first check due to
    ** the link being in sleep state, call it again,
    ** if it fails a second time its a real issue.
    */
    if (e1000_validate_nvm_checksum(&adapter->hw) < 0) {
      device_printf(dev,
                    "The EEPROM Checksum Is Not Valid\n");
      error = EIO;
      goto err_hw_init;
    }
  }

  /* Copy the permanent MAC address out of the EEPROM */
  if (e1000_read_mac_addr(&adapter->hw) < 0) {
    device_printf(dev, "EEPROM read error while reading MAC"
                  " address\n");
    error = EIO;
    goto err_hw_init;
  }

  if (!lem_is_valid_ether_addr(adapter->hw.mac.addr)) {
    device_printf(dev, "Invalid MAC address\n");
    error = EIO;
    goto err_hw_init;
  }

  /* Initialize the hardware */
  if (lem_hardware_init(adapter)) {
    device_printf(dev, "Unable to initialize the hardware\n");
    error = EIO;
    goto err_hw_init;
  }

  /* Allocate transmit descriptors and buffers */
  if (lem_allocate_transmit_structures(adapter)) {
    device_printf(dev, "Could not setup transmit structures\n");
    error = ENOMEM;
    goto err_tx_struct;
  }

  /* Allocate receive descriptors and buffers */
  if (lem_allocate_receive_structures(adapter)) {
    device_printf(dev, "Could not setup receive structures\n");
    error = ENOMEM;
    goto err_rx_struct;
  }

  /*
  **  Do interrupt configuration
  */
  error = lem_allocate_irq(adapter);
  if (error)
    goto err_rx_struct;

  /*
   * Get Wake-on-Lan and Management info for later use
   */
  lem_get_wakeup(dev);

  /* Setup OS specific network interface */
  if (lem_setup_interface(dev, adapter) != 0)
    goto err_rx_struct;

  /* Initialize statistics */
  lem_update_stats_counters(adapter);

  adapter->hw.mac.get_link_status = 1;
  lem_update_link_status(adapter);

  /* Indicate SOL/IDER usage */
  if (e1000_check_reset_block(&adapter->hw))
    device_printf(dev,
                  "PHY reset is blocked due to SOL/IDER session.\n");

  /* Do we need workaround for 82544 PCI-X adapter? */
  if (adapter->hw.bus.type == e1000_bus_type_pcix &&
      adapter->hw.mac.type == e1000_82544)
    adapter->pcix_82544 = TRUE;
  else
    adapter->pcix_82544 = FALSE;

  // 	/* Register for VLAN events */
  // 	adapter->vlan_attach = EVENTHANDLER_REGISTER(vlan_config,
  // 	    lem_register_vlan, adapter, EVENTHANDLER_PRI_FIRST);
  // 	adapter->vlan_detach = EVENTHANDLER_REGISTER(vlan_unconfig,
  // 	    lem_unregister_vlan, adapter, EVENTHANDLER_PRI_FIRST); 

  // 	lem_add_hw_stats(adapter);

  /* Non-AMT based hardware can now take control from firmware */
  if (adapter->has_manage && !adapter->has_amt)
    lem_get_hw_control(adapter);

  /* Tell the stack that the interface is not active */
  if_setdrvflagbits(adapter->ifp, 0, IFF_DRV_OACTIVE | IFF_DRV_RUNNING);

  adapter->led_dev = NULL; // led_create(lem_led_func, adapter,
                           //      device_get_nameunit(dev));

  // #ifdef DEV_NETMAP
  // 	lem_netmap_attach(adapter);
  // #endif /* DEV_NETMAP */
  INIT_DEBUGOUT("lem_attach: end");

  return (0);

  err_rx_struct:
  //  	lem_free_transmit_structures(adapter);
  err_tx_struct:
  err_hw_init:
  //	lem_release_hw_control(adapter);
  //	lem_dma_free(adapter, &adapter->rxdma);
  err_rx_desc:
  //	lem_dma_free(adapter, &adapter->txdma);
  err_tx_desc:
  // #ifdef NIC_PARAVIRT
  // 	lem_dma_free(adapter, &adapter->csb_mem);
  // err_csb:
  // #endif /* NIC_PARAVIRT */

  err_pci:
  // 	if (adapter->ifp != (void *)NULL)
  // 		if_free(adapter->ifp);
  // 	lem_free_pci_resources(adapter);
  // 	free(adapter->mta, M_DEVBUF);
  EM_TX_LOCK_DESTROY(adapter);
  EM_RX_LOCK_DESTROY(adapter);
  EM_CORE_LOCK_DESTROY(adapter);

  return (error);
}

/*********************************************************************
 *
 *  Determine hardware revision.
 *
 **********************************************************************/
static void
lem_identify_hardware(struct adapter *adapter)
{
  device_t dev = adapter->dev;

  /* Make sure our PCI config space has the necessary stuff set */
  pci_enable_busmaster(dev);
  adapter->hw.bus.pci_cmd_word = pci_read_config(dev, PCIR_COMMAND, 2);

  /* Save off the information about this board */
  adapter->hw.vendor_id = pci_get_vendor(dev);
  adapter->hw.device_id = pci_get_device(dev);
  adapter->hw.revision_id = pci_read_config(dev, PCIR_REVID, 1);
  adapter->hw.subsystem_vendor_id =
    pci_read_config(dev, PCIR_SUBVEND_0, 2);
  adapter->hw.subsystem_device_id =
    pci_read_config(dev, PCIR_SUBDEV_0, 2);

  /* Do Shared Code Init and Setup */
  if (e1000_set_mac_type(&adapter->hw)) {
    device_printf(dev, "Setup init failure\n");
    return;
  }
}

static int
lem_allocate_pci_resources(struct adapter *adapter)
{
  device_t	dev = adapter->dev;
  int		val, rid, error = E1000_SUCCESS;

  rid = PCIR_BAR(0);
  adapter->memory = bus_alloc_resource_from_bar(dev, rid);
  if (adapter->memory == NULL) {
    device_printf(dev, "Unable to allocate bus resource: memory\n");
    return (ENXIO);
  }
  adapter->osdep.mem_bus_space_tag = 
    rman_get_bustag(adapter->memory);
  adapter->osdep.mem_bus_space_handle =
    rman_get_bushandle(adapter->memory);
  adapter->hw.hw_addr = (u8 *)&adapter->osdep.mem_bus_space_handle;

  /* Only older adapters use IO mapping */
  if (adapter->hw.mac.type > e1000_82543) {
    /* Figure our where our IO BAR is ? */
    for (rid = PCIR_BAR(0); rid < PCIR_CIS;) {
      val = pci_read_config(dev, rid, 4);
      if (EM_BAR_TYPE(val) == EM_BAR_TYPE_IO) {
        adapter->io_rid = rid;
        break;
      }
      rid += 4;
      /* check for 64bit BAR */
      if (EM_BAR_MEM_TYPE(val) == EM_BAR_MEM_TYPE_64BIT)
        rid += 4;
    }
    if (rid >= PCIR_CIS) {
      device_printf(dev, "Unable to locate IO BAR\n");
      return (ENXIO);
    }
    adapter->ioport = bus_alloc_resource_from_bar(dev, adapter->io_rid);
    if (adapter->ioport == NULL) {
      device_printf(dev, "Unable to allocate bus resource: "
                    "ioport\n");
      return (ENXIO);
    }
    adapter->hw.io_base = 0;
    adapter->osdep.io_bus_space_tag =
      rman_get_bustag(adapter->ioport);
    adapter->osdep.io_bus_space_handle =
      rman_get_bushandle(adapter->ioport);
  }
        
  adapter->hw.back = &adapter->osdep;

  return (error);
}

static int
lem_dma_malloc(struct adapter *adapter, bus_size_t size,
        struct em_dma_alloc *dma, int mapflags)
{
  PhysAddr paddr;
  physmem_ctrl->Alloc(paddr, PagingCtrl::RoundUpAddrOnPageBoundary(size));
  dma->dma_paddr = reinterpret_cast<bus_addr_t>(paddr.GetAddr());
  dma->dma_vaddr = reinterpret_cast<caddr_t>(p2v(paddr.GetAddr()));
  
  return (0);
}

static int
lem_is_valid_ether_addr(u8 *addr)
{
	char zero_addr[6] = { 0, 0, 0, 0, 0, 0 };

	if ((addr[0] & 1) || (!bcmp(addr, zero_addr, ETHER_ADDR_LEN))) {
		return (FALSE);
	}

	return (TRUE);
}

/*********************************************************************
 *
 *  Initialize the hardware to a configuration
 *  as specified by the adapter structure.
 *
 **********************************************************************/
static int
lem_hardware_init(struct adapter *adapter)
{
	device_t dev = adapter->dev;
	u16 	rx_buffer_size;

	INIT_DEBUGOUT("lem_hardware_init: begin");

	/* Issue a global reset */
	e1000_reset_hw(&adapter->hw);

	/* When hardware is reset, fifo_head is also reset */
	adapter->tx_fifo_head = 0;

	/*
	 * These parameters control the automatic generation (Tx) and
	 * response (Rx) to Ethernet PAUSE frames.
	 * - High water mark should allow for at least two frames to be
	 *   received after sending an XOFF.
	 * - Low water mark works best when it is very near the high water mark.
	 *   This allows the receiver to restart by sending XON when it has
	 *   drained a bit. Here we use an arbitary value of 1500 which will
	 *   restart after one full frame is pulled from the buffer. There
	 *   could be several smaller frames in the buffer and if so they will
	 *   not trigger the XON until their total number reduces the buffer
	 *   by 1500.
	 * - The pause time is fairly large at 1000 x 512ns = 512 usec.
	 */
	rx_buffer_size = ((E1000_READ_REG(&adapter->hw, E1000_PBA) &
	    0xffff) << 10 );

	adapter->hw.fc.high_water = rx_buffer_size -
	    roundup2(adapter->max_frame_size, 1024);
	adapter->hw.fc.low_water = adapter->hw.fc.high_water - 1500;

	adapter->hw.fc.pause_time = EM_FC_PAUSE_TIME;
	adapter->hw.fc.send_xon = TRUE;

        /* Set Flow control, use the tunable location if sane */
        if ((lem_fc_setting >= 0) && (lem_fc_setting < 4))
                adapter->hw.fc.requested_mode = lem_fc_setting;
        else
                adapter->hw.fc.requested_mode = e1000_fc_none;

	if (e1000_init_hw(&adapter->hw) < 0) {
		device_printf(dev, "Hardware Initialization Failed\n");
		return (EIO);
	}

	e1000_check_for_link(&adapter->hw);

	return (0);
}

/*********************************************************************
 *
 *  Allocate memory for tx_buffer structures. The tx_buffer stores all
 *  the information needed to transmit a packet on the wire.
 *
 **********************************************************************/
static int
lem_allocate_transmit_structures(struct adapter *adapter)
{
	device_t dev = adapter->dev;
	struct em_buffer *tx_buffer;
	int error;
        E1000 *e1000 = dev->parent;

	/*
	 * Create DMA tags for tx descriptors
	 */
	if ((error = bus_dma_tag_create(bus_get_dma_tag(dev), /* parent */
				1, 0,			/* alignment, bounds */
				BUS_SPACE_MAXADDR,	/* lowaddr */
				BUS_SPACE_MAXADDR,	/* highaddr */
				NULL, NULL,		/* filter, filterarg */
				MCLBYTES * EM_MAX_SCATTER,	/* maxsize */
				EM_MAX_SCATTER,		/* nsegments */
				MCLBYTES,		/* maxsegsize */
				0,			/* flags */
				NULL,			/* lockfunc */
				NULL,			/* lockarg */
				&adapter->txtag)) != 0) {
		device_printf(dev, "Unable to allocate TX DMA tag\n");
		goto fail;
	}

	adapter->tx_buffer_area = reinterpret_cast<struct em_buffer *>(virtmem_ctrl->Alloc(sizeof(struct em_buffer) * adapter->num_tx_desc));
	if (adapter->tx_buffer_area == NULL) {
		device_printf(dev, "Unable to allocate tx_buffer memory\n");
		error = ENOMEM;
		goto fail;
	}

	/* Create the descriptor buffer dma maps */
	for (int i = 0; i < adapter->num_tx_desc; i++) {
		tx_buffer = &adapter->tx_buffer_area[i];
		error = bus_dmamap_create(adapter->txtag, 0, &tx_buffer->map);
		if (error != 0) {
			device_printf(dev, "Unable to create TX DMA map\n");
			goto fail;
		}
		tx_buffer->next_eop = -1;
	}

        while(!e1000->_tx_reserved.IsFull()) {
          E1000::Packet *packet = reinterpret_cast<E1000::Packet *>(virtmem_ctrl->Alloc(sizeof(E1000::Packet)));
          kassert(e1000->_tx_reserved.Push(packet));
        }

	return (0);
fail:
	// lem_free_transmit_structures(adapter);
	return (error);
}

/*********************************************************************
 *
 *  Allocate memory for rx_buffer structures. Since we use one
 *  rx_buffer per received packet, the maximum number of rx_buffer's
 *  that we'll need is equal to the number of receive descriptors
 *  that we've allocated.
 *
 **********************************************************************/
static int
lem_allocate_receive_structures(struct adapter *adapter)
{
	device_t dev = adapter->dev;
	struct em_buffer *rx_buffer;
	int i, error;
        E1000 *e1000 = dev->parent;

	adapter->rx_buffer_area = reinterpret_cast<struct em_buffer *>(virtmem_ctrl->Alloc(sizeof(struct em_buffer) * adapter->num_rx_desc));
	if (adapter->rx_buffer_area == NULL) {
		device_printf(dev, "Unable to allocate rx_buffer memory\n");
		return (ENOMEM);
	}

	error = bus_dma_tag_create(bus_get_dma_tag(dev), /* parent */
				1, 0,			/* alignment, bounds */
				BUS_SPACE_MAXADDR,	/* lowaddr */
				BUS_SPACE_MAXADDR,	/* highaddr */
				NULL, NULL,		/* filter, filterarg */
				MCLBYTES,		/* maxsize */
				1,			/* nsegments */
				MCLBYTES,		/* maxsegsize */
				0,			/* flags */
				NULL,			/* lockfunc */
				NULL,			/* lockarg */
				&adapter->rxtag);
	if (error) {
		device_printf(dev, "%s: bus_dma_tag_create failed %d\n",
		    __func__, error);
		goto fail;
	}

	/* Create the spare map (used by getbuf) */
	error = bus_dmamap_create(adapter->rxtag, 0, &adapter->rx_sparemap);
	if (error) {
		device_printf(dev, "%s: bus_dmamap_create failed: %d\n",
		    __func__, error);
		goto fail;
	}

	rx_buffer = adapter->rx_buffer_area;
	for (i = 0; i < adapter->num_rx_desc; i++, rx_buffer++) {
		error = bus_dmamap_create(adapter->rxtag, 0, &rx_buffer->map);
		if (error) {
			device_printf(dev, "%s: bus_dmamap_create failed: %d\n",
			    __func__, error);
			goto fail;
		}
	}

        while(!e1000->_rx_reserved.IsFull()) {
          E1000::Packet *packet = reinterpret_cast<E1000::Packet *>(virtmem_ctrl->Alloc(sizeof(E1000::Packet)));
          kassert(e1000->_rx_reserved.Push(packet));
        }

	return (0);

fail:
	// lem_free_receive_structures(adapter);
	return (error);
}

/*********************************************************************
 *
 *  Setup the Legacy or MSI Interrupt handler
 *
 **********************************************************************/
int
lem_allocate_irq(struct adapter *adapter)
{
	device_t dev = adapter->dev;
	int error, rid = 0;

	/* Manually turn off all interrupts */
	E1000_WRITE_REG(&adapter->hw, E1000_IMC, 0xffffffff);

	/* We allocate a single interrupt resource */
	adapter->res[0] = bus_alloc_resource_from_bar(dev, rid);
	if (adapter->res[0] == NULL) {
		device_printf(dev, "Unable to allocate bus resource: "
		    "interrupt\n");
		return (ENXIO);
	}

	/* Do Legacy setup? */
	if (lem_use_legacy_irq) {
		// if ((error = bus_setup_intr(dev, adapter->res[0],
	    	//     INTR_TYPE_NET | INTR_MPSAFE, NULL, lem_intr, adapter,
	    	//     &adapter->tag[0])) != 0) {
		// 	device_printf(dev,
		// 	    "Failed to register interrupt handler");
		// 	return (error);
		// }
		return (0);
	}

	/*
	 * Use a Fast interrupt and the associated
	 * deferred processing contexts.
	 */
	// TASK_INIT(&adapter->rxtx_task, 0, lem_handle_rxtx, adapter);
	// TASK_INIT(&adapter->link_task, 0, lem_handle_link, adapter);
	// adapter->tq = taskqueue_create_fast("lem_taskq", M_NOWAIT,
	//     taskqueue_thread_enqueue, &adapter->tq);
	// taskqueue_start_threads(&adapter->tq, 1, PI_NET, "%s taskq",
	//     device_get_nameunit(adapter->dev));
	// if ((error = bus_setup_intr(dev, adapter->res[0],
	//     INTR_TYPE_NET, lem_irq_fast, NULL, adapter,
	//     &adapter->tag[0])) != 0) {
	// 	device_printf(dev, "Failed to register fast interrupt "
	// 		    "handler: %d\n", error);
	// 	taskqueue_free(adapter->tq);
	// 	adapter->tq = NULL;
	// 	return (error);
	// }
	
	return (0);
}

/*
** Parse the interface capabilities with regard
** to both system management and wake-on-lan for
** later use.
*/
static void
lem_get_wakeup(device_t dev)
{
	struct adapter	*adapter = device_get_softc(dev);
	u16		eeprom_data = 0, device_id, apme_mask;

	adapter->has_manage = e1000_enable_mng_pass_thru(&adapter->hw);
	apme_mask = EM_EEPROM_APME;

	switch (adapter->hw.mac.type) {
	case e1000_82542:
	case e1000_82543:
		break;
	case e1000_82544:
		e1000_read_nvm(&adapter->hw,
		    NVM_INIT_CONTROL2_REG, 1, &eeprom_data);
		apme_mask = EM_82544_APME;
		break;
	case e1000_82546:
	case e1000_82546_rev_3:
		if (adapter->hw.bus.func == 1) {
			e1000_read_nvm(&adapter->hw,
			    NVM_INIT_CONTROL3_PORT_B, 1, &eeprom_data);
			break;
		} else
			e1000_read_nvm(&adapter->hw,
			    NVM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		break;
	default:
		e1000_read_nvm(&adapter->hw,
		    NVM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		break;
	}
	if (eeprom_data & apme_mask)
		adapter->wol = (E1000_WUFC_MAG | E1000_WUFC_MC);
	/*
         * We have the eeprom settings, now apply the special cases
         * where the eeprom may be wrong or the board won't support
         * wake on lan on a particular port
	 */
	device_id = pci_get_device(dev);
        switch (device_id) {
	case E1000_DEV_ID_82546GB_PCIE:
		adapter->wol = 0;
		break;
	case E1000_DEV_ID_82546EB_FIBER:
	case E1000_DEV_ID_82546GB_FIBER:
		/* Wake events only supported on port A for dual fiber
		 * regardless of eeprom setting */
		if (E1000_READ_REG(&adapter->hw, E1000_STATUS) &
		    E1000_STATUS_FUNC_1)
			adapter->wol = 0;
		break;
	case E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3:
                /* if quad port adapter, disable WoL on all but port A */
		if (global_quad_port_a != 0)
			adapter->wol = 0;
		/* Reset for multiple quad port adapters */
		if (++global_quad_port_a == 4)
			global_quad_port_a = 0;
                break;
	}
	return;
}


/**********************************************************************
 *
 *  Update the board statistics counters.
 *
 **********************************************************************/
static void
lem_update_stats_counters(struct adapter *adapter)
{

	if(adapter->hw.phy.media_type == e1000_media_type_copper ||
	   (E1000_READ_REG(&adapter->hw, E1000_STATUS) & E1000_STATUS_LU)) {
		adapter->stats.symerrs += E1000_READ_REG(&adapter->hw, E1000_SYMERRS);
		adapter->stats.sec += E1000_READ_REG(&adapter->hw, E1000_SEC);
	}
	adapter->stats.crcerrs += E1000_READ_REG(&adapter->hw, E1000_CRCERRS);
	adapter->stats.mpc += E1000_READ_REG(&adapter->hw, E1000_MPC);
	adapter->stats.scc += E1000_READ_REG(&adapter->hw, E1000_SCC);
	adapter->stats.ecol += E1000_READ_REG(&adapter->hw, E1000_ECOL);

	adapter->stats.mcc += E1000_READ_REG(&adapter->hw, E1000_MCC);
	adapter->stats.latecol += E1000_READ_REG(&adapter->hw, E1000_LATECOL);
	adapter->stats.colc += E1000_READ_REG(&adapter->hw, E1000_COLC);
	adapter->stats.dc += E1000_READ_REG(&adapter->hw, E1000_DC);
	adapter->stats.rlec += E1000_READ_REG(&adapter->hw, E1000_RLEC);
	adapter->stats.xonrxc += E1000_READ_REG(&adapter->hw, E1000_XONRXC);
	adapter->stats.xontxc += E1000_READ_REG(&adapter->hw, E1000_XONTXC);
	adapter->stats.xoffrxc += E1000_READ_REG(&adapter->hw, E1000_XOFFRXC);
	adapter->stats.xofftxc += E1000_READ_REG(&adapter->hw, E1000_XOFFTXC);
	adapter->stats.fcruc += E1000_READ_REG(&adapter->hw, E1000_FCRUC);
	adapter->stats.prc64 += E1000_READ_REG(&adapter->hw, E1000_PRC64);
	adapter->stats.prc127 += E1000_READ_REG(&adapter->hw, E1000_PRC127);
	adapter->stats.prc255 += E1000_READ_REG(&adapter->hw, E1000_PRC255);
	adapter->stats.prc511 += E1000_READ_REG(&adapter->hw, E1000_PRC511);
	adapter->stats.prc1023 += E1000_READ_REG(&adapter->hw, E1000_PRC1023);
	adapter->stats.prc1522 += E1000_READ_REG(&adapter->hw, E1000_PRC1522);
	adapter->stats.gprc += E1000_READ_REG(&adapter->hw, E1000_GPRC);
	adapter->stats.bprc += E1000_READ_REG(&adapter->hw, E1000_BPRC);
	adapter->stats.mprc += E1000_READ_REG(&adapter->hw, E1000_MPRC);
	adapter->stats.gptc += E1000_READ_REG(&adapter->hw, E1000_GPTC);

	/* For the 64-bit byte counters the low dword must be read first. */
	/* Both registers clear on the read of the high dword */

	adapter->stats.gorc += E1000_READ_REG(&adapter->hw, E1000_GORCL) +
	    ((u64)E1000_READ_REG(&adapter->hw, E1000_GORCH) << 32);
	adapter->stats.gotc += E1000_READ_REG(&adapter->hw, E1000_GOTCL) +
	    ((u64)E1000_READ_REG(&adapter->hw, E1000_GOTCH) << 32);

	adapter->stats.rnbc += E1000_READ_REG(&adapter->hw, E1000_RNBC);
	adapter->stats.ruc += E1000_READ_REG(&adapter->hw, E1000_RUC);
	adapter->stats.rfc += E1000_READ_REG(&adapter->hw, E1000_RFC);
	adapter->stats.roc += E1000_READ_REG(&adapter->hw, E1000_ROC);
	adapter->stats.rjc += E1000_READ_REG(&adapter->hw, E1000_RJC);

	adapter->stats.tor += E1000_READ_REG(&adapter->hw, E1000_TORH);
	adapter->stats.tot += E1000_READ_REG(&adapter->hw, E1000_TOTH);

	adapter->stats.tpr += E1000_READ_REG(&adapter->hw, E1000_TPR);
	adapter->stats.tpt += E1000_READ_REG(&adapter->hw, E1000_TPT);
	adapter->stats.ptc64 += E1000_READ_REG(&adapter->hw, E1000_PTC64);
	adapter->stats.ptc127 += E1000_READ_REG(&adapter->hw, E1000_PTC127);
	adapter->stats.ptc255 += E1000_READ_REG(&adapter->hw, E1000_PTC255);
	adapter->stats.ptc511 += E1000_READ_REG(&adapter->hw, E1000_PTC511);
	adapter->stats.ptc1023 += E1000_READ_REG(&adapter->hw, E1000_PTC1023);
	adapter->stats.ptc1522 += E1000_READ_REG(&adapter->hw, E1000_PTC1522);
	adapter->stats.mptc += E1000_READ_REG(&adapter->hw, E1000_MPTC);
	adapter->stats.bptc += E1000_READ_REG(&adapter->hw, E1000_BPTC);

	if (adapter->hw.mac.type >= e1000_82543) {
		adapter->stats.algnerrc += 
		E1000_READ_REG(&adapter->hw, E1000_ALGNERRC);
		adapter->stats.rxerrc += 
		E1000_READ_REG(&adapter->hw, E1000_RXERRC);
		adapter->stats.tncrs += 
		E1000_READ_REG(&adapter->hw, E1000_TNCRS);
		adapter->stats.cexterr += 
		E1000_READ_REG(&adapter->hw, E1000_CEXTERR);
		adapter->stats.tsctc += 
		E1000_READ_REG(&adapter->hw, E1000_TSCTC);
		adapter->stats.tsctfc += 
		E1000_READ_REG(&adapter->hw, E1000_TSCTFC);
	}
}

static void
lem_update_link_status(struct adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	if_t ifp = adapter->ifp;
	device_t dev = adapter->dev;
	u32 link_check = 0;

	/* Get the cached link value or read phy for real */
	switch (hw->phy.media_type) {
	case e1000_media_type_copper:
		if (hw->mac.get_link_status) {
			/* Do the work to read phy */
			e1000_check_for_link(hw);
			link_check = !hw->mac.get_link_status;
			if (link_check) /* ESB2 fix */
				e1000_cfg_on_link_up(hw);
		} else
			link_check = TRUE;
		break;
	case e1000_media_type_fiber:
		e1000_check_for_link(hw);
		link_check = (E1000_READ_REG(hw, E1000_STATUS) &
                                 E1000_STATUS_LU);
		break;
	case e1000_media_type_internal_serdes:
		e1000_check_for_link(hw);
		link_check = adapter->hw.mac.serdes_has_link;
		break;
	default:
	case e1000_media_type_unknown:
		break;
	}

	/* Now check for a transition */
	if (link_check && (adapter->link_active == 0)) {
		e1000_get_speed_and_duplex(hw, &adapter->link_speed,
		    &adapter->link_duplex);
		if (bootverbose)
			device_printf(dev, "Link is up %d Mbps %s\n",
			    adapter->link_speed,
			    ((adapter->link_duplex == FULL_DUPLEX) ?
			    "Full Duplex" : "Half Duplex"));
		adapter->link_active = 1;
		adapter->smartspeed = 0;
		// if_setbaudrate(ifp, adapter->link_speed * 1000000);
		// if_link_state_change(ifp, LINK_STATE_UP);
	} else if (!link_check && (adapter->link_active == 1)) {
		// if_setbaudrate(ifp, 0);
		adapter->link_speed = 0;
		adapter->link_duplex = 0;
		if (bootverbose)
			device_printf(dev, "Link is Down\n");
		adapter->link_active = 0;
		/* Link down, disable watchdog */
		adapter->watchdog_check = FALSE;
		// if_link_state_change(ifp, LINK_STATE_DOWN);
	}
}

/*
 * lem_get_hw_control sets the {CTRL_EXT|FWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means
 * that the driver is loaded. For AMT version type f/w
 * this means that the network i/f is open.
 */
static void
lem_get_hw_control(struct adapter *adapter)
{
	u32 ctrl_ext;

	ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
	    ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
	return;
}

/*********************************************************************
 *
 *  Setup networking device structure and register an interface.
 *
 **********************************************************************/
static int
lem_setup_interface(device_t dev, struct adapter *adapter)
{
  if_t ifp;

  INIT_DEBUGOUT("lem_setup_interface: begin");

  ifp = adapter->ifp = if_gethandle(IFT_ETHER);
  if (ifp == (void *)NULL) {
    device_printf(dev, "can not allocate ifnet structure\n");
    return (-1);
  }
  // 	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
  // 	if_setinitfn(ifp,  lem_init);
  if_setsoftc(ifp, adapter);
  // 	if_setflags(ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
  // 	if_setioctlfn(ifp, lem_ioctl);
  // 	if_setstartfn(ifp, lem_start);
  // 	if_setsendqlen(ifp, adapter->num_tx_desc - 1);
  // 	if_setsendqready(ifp);
  // #if __FreeBSD_version >= 1100036
  // 	if_setgetcounterfn(ifp, lem_get_counter);
  // #endif

  // 	ether_ifattach(ifp, adapter->hw.mac.addr);

  // 	if_setcapabilities(ifp, 0);

  // 	if (adapter->hw.mac.type >= e1000_82543) {
  // 		if_setcapabilitiesbit(ifp, IFCAP_HWCSUM | IFCAP_VLAN_HWCSUM, 0);
  // 		if_setcapenablebit(ifp, IFCAP_HWCSUM | IFCAP_VLAN_HWCSUM, 0);
  // 	}

  // 	/*
  // 	 * Tell the upper layer(s) we support long frames.
  // 	 */
  // 	if_setifheaderlen(ifp, sizeof(struct ether_vlan_header));
  // 	if_setcapabilitiesbit(ifp, IFCAP_VLAN_HWTAGGING | IFCAP_VLAN_MT,U 0);
  // 	if_setcapenablebit(ifp, IFCAP_VLAN_HWTAGGING | IFCAP_VLAN_MTU, 0);

  // 	/*
  // 	** Dont turn this on by default, if vlans are
  // 	** created on another pseudo device (eg. lagg)
  // 	** then vlan events are not passed thru, breaking
  // 	** operation, but with HW FILTER off it works. If
  // 	** using vlans directly on the em driver you can
  // 	** enable this and get full hardware tag filtering.
  // 	*/
  // 	if_setcapabilitiesbit(ifp, IFCAP_VLAN_HWFILTER, 0);

  // #ifdef DEVICE_POLLING
  // 	if_setcapabilitiesbit(ifp, IFCAP_POLLING, 0);
  // #endif

  // 	/* Enable only WOL MAGIC by default */
  // 	if (adapter->wol) {
  // 		if_setcapabilitiesbit(ifp, IFCAP_WOL, 0);
  // 		if_setcapenablebit(ifp, IFCAP_WOL_MAGIC, 0);
  // 	}
		
  // 	/*
  // 	 * Specify the media types supported by this adapter and register
  // 	 * callbacks to update media and link information
  // 	 */
  // 	ifmedia_init(&adapter->media, IFM_IMASK,
  // 	    lem_media_change, lem_media_status);
  // 	if ((adapter->hw.phy.media_type == e1000_media_type_fiber) ||
  // 	    (adapter->hw.phy.media_type == e1000_media_type_internal_serdes)) {
  // 		u_char fiber_type = IFM_1000_SX;	/* default type */

  // 		if (adapter->hw.mac.type == e1000_82545)
  // 			fiber_type = IFM_1000_LX;
  // 		ifmedia_add(&adapter->media, IFM_ETHER | fiber_type | IFM_FDX, 
  // 			    0, NULL);
  // 		ifmedia_add(&adapter->media, IFM_ETHER | fiber_type, 0, NULL);
  // 	} else {
  // 		ifmedia_add(&adapter->media, IFM_ETHER | IFM_10_T, 0, NULL);
  // 		ifmedia_add(&adapter->media, IFM_ETHER | IFM_10_T | IFM_FDX,
  // 			    0, NULL);
  // 		ifmedia_add(&adapter->media, IFM_ETHER | IFM_100_TX,
  // 			    0, NULL);
  // 		ifmedia_add(&adapter->media, IFM_ETHER | IFM_100_TX | IFM_FDX,
  // 			    0, NULL);
  // 		if (adapter->hw.phy.type != e1000_phy_ife) {
  // 			ifmedia_add(&adapter->media,
  // 				IFM_ETHER | IFM_1000_T | IFM_FDX, 0, NULL);
  // 			ifmedia_add(&adapter->media,
  // 				IFM_ETHER | IFM_1000_T, 0, NULL);
  // 		}
  // 	}
  // 	ifmedia_add(&adapter->media, IFM_ETHER | IFM_AUTO, 0, NULL);
  // 	ifmedia_set(&adapter->media, IFM_ETHER | IFM_AUTO);

  return (0);
}

/*********************************************************************
 *
 *  Legacy polling routine  
 *
 *********************************************************************/
static int
lem_poll(if_t ifp)
{
  struct adapter *adapter = reinterpret_cast<struct adapter *>(if_getsoftc(ifp));
  E1000 *e1000 = adapter->dev->parent;
	u32		reg_icr;
        int rx_done = 0;

	EM_CORE_LOCK(adapter);
	if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) == 0) {
		EM_CORE_UNLOCK(adapter);
		return (rx_done);
	}

        // TODO : 要対応
 	// if (cmd == POLL_AND_CHECK_STATUS) {
	// 	reg_icr = E1000_READ_REG(&adapter->hw, E1000_ICR);
	// 	if (reg_icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
	// 		callout_stop(&adapter->timer);
	// 		adapter->hw.mac.get_link_status = 1;
	// 		lem_update_link_status(adapter);
	// 		callout_reset(&adapter->timer, hz,
	// 		    lem_local_timer, adapter);
	// 	}
	// }
	EM_CORE_UNLOCK(adapter);

        int count = 1;
	lem_rxeof(adapter, count, &rx_done);

	EM_TX_LOCK(adapter);
	lem_txeof(adapter);

        // if(!if_sendq_empty(ifp))
        if (!e1000->_tx_buffered.IsEmpty())
          lem_start_locked(ifp);
        EM_TX_UNLOCK(adapter);
	return (rx_done);
}

static void
lem_disable_intr(struct adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;

	E1000_WRITE_REG(hw, E1000_IMC, 0xffffffff);
}

static void
lem_init(struct adapter *arg)
{
	struct adapter *adapter = arg;

	EM_CORE_LOCK(adapter);
	lem_init_locked(adapter);
	EM_CORE_UNLOCK(adapter);
}

static void
lem_init_locked(struct adapter *adapter)
{
	if_t ifp = adapter->ifp;
	device_t	dev = adapter->dev;
	u32		pba;

	INIT_DEBUGOUT("lem_init: begin");

	EM_CORE_LOCK_ASSERT(adapter);

	EM_TX_LOCK(adapter);
	lem_stop(adapter);
	EM_TX_UNLOCK(adapter);

	/*
	 * Packet Buffer Allocation (PBA)
	 * Writing PBA sets the receive portion of the buffer
	 * the remainder is used for the transmit buffer.
	 *
	 * Devices before the 82547 had a Packet Buffer of 64K.
	 *   Default allocation: PBA=48K for Rx, leaving 16K for Tx.
	 * After the 82547 the buffer was reduced to 40K.
	 *   Default allocation: PBA=30K for Rx, leaving 10K for Tx.
	 *   Note: default does not leave enough room for Jumbo Frame >10k.
	 */
	switch (adapter->hw.mac.type) {
	case e1000_82547:
	case e1000_82547_rev_2: /* 82547: Total Packet Buffer is 40K */
		if (adapter->max_frame_size > 8192)
			pba = E1000_PBA_22K; /* 22K for Rx, 18K for Tx */
		else
			pba = E1000_PBA_30K; /* 30K for Rx, 10K for Tx */
		adapter->tx_fifo_head = 0;
		adapter->tx_head_addr = pba << EM_TX_HEAD_ADDR_SHIFT;
		adapter->tx_fifo_size =
		    (E1000_PBA_40K - pba) << EM_PBA_BYTES_SHIFT;
		break;
	default:
		/* Devices before 82547 had a Packet Buffer of 64K.   */
		if (adapter->max_frame_size > 8192)
			pba = E1000_PBA_40K; /* 40K for Rx, 24K for Tx */
		else
			pba = E1000_PBA_48K; /* 48K for Rx, 16K for Tx */
	}

	INIT_DEBUGOUT1("lem_init: pba=%dK",pba);
	E1000_WRITE_REG(&adapter->hw, E1000_PBA, pba);
	
	/* Get the latest mac address, User can use a LAA */
        // bcopy(if_getlladdr(adapter->ifp), adapter->hw.mac.addr,
        //       ETHER_ADDR_LEN);

	/* Put the address into the Receive Address Array */
	e1000_rar_set(&adapter->hw, adapter->hw.mac.addr, 0);

	/* Initialize the hardware */
	if (lem_hardware_init(adapter)) {
		device_printf(dev, "Unable to initialize the hardware\n");
		return;
	}
        lem_update_link_status(adapter);

	/* Setup VLAN support, basic and offload if available */
        // E1000_WRITE_REG(&adapter->hw, E1000_VET, ETHERTYPE_VLAN);

	/* Set hardware offload abilities */
	// if_clearhwassist(ifp);
	// if (adapter->hw.mac.type >= e1000_82543) {
	// 	if (if_getcapenable(ifp) & IFCAP_TXCSUM)
	// 		if_sethwassistbits(ifp, CSUM_TCP | CSUM_UDP, 0);
	// }

	/* Configure for OS presence */
        lem_init_manageability(adapter);

	/* Prepare transmit descriptors and buffers */
        lem_setup_transmit_structures(adapter);
        lem_initialize_transmit_unit(adapter);

	/* Setup Multicast table */
        lem_set_multi(adapter);

	/* Prepare receive descriptors and buffers */
	if (lem_setup_receive_structures(adapter)) {
		device_printf(dev, "Could not setup receive structures\n");
		EM_TX_LOCK(adapter);
		lem_stop(adapter);
		EM_TX_UNLOCK(adapter);
		return;
	}
        lem_initialize_receive_unit(adapter);

	/* Use real VLAN Filter support? */
	// if (if_getcapenable(ifp) & IFCAP_VLAN_HWTAGGING) {
	// 	if (if_getcapenable(ifp) & IFCAP_VLAN_HWFILTER)
	// 		/* Use real VLAN Filter support */
	// 		lem_setup_vlan_hw_support(adapter);
	// 	else {
	// 		u32 ctrl;
	// 		ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
	// 		ctrl |= E1000_CTRL_VME;
	// 		E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);
        //         }
	// }

	/* Don't lose promiscuous settings */
	// lem_set_promisc(adapter);

	if_setdrvflagbits(ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);

	callout_reset(&adapter->timer, hz, lem_local_timer, adapter);
	e1000_clear_hw_cntrs_base_generic(&adapter->hw);

	// /*
	//  * Only enable interrupts if we are not polling, make sure
	//  * they are off otherwise.
	//  */
	// if (if_getcapenable(ifp) & IFCAP_POLLING)
        lem_disable_intr(adapter);
	// else
	// 	lem_enable_intr(adapter);

	/* AMT based hardware can now take control from firmware */
        if (adapter->has_manage && adapter->has_amt)
          lem_get_hw_control(adapter);
}

/*********************************************************************
 *
 *  This routine disables all traffic on the adapter by issuing a
 *  global reset on the MAC and deallocates TX/RX buffers.
 *
 *  This routine should always be called with BOTH the CORE
 *  and TX locks.
 **********************************************************************/

static void
lem_stop(struct adapter *arg)
{
	struct adapter	*adapter = arg;
	if_t ifp = adapter->ifp;

	EM_CORE_LOCK_ASSERT(adapter);
	EM_TX_LOCK_ASSERT(adapter);

	INIT_DEBUGOUT("lem_stop: begin");

	lem_disable_intr(adapter);
	callout_stop(&adapter->timer);
	callout_stop(&adapter->tx_fifo_timer);

	/* Tell the stack that the interface is no longer active */
	if_setdrvflagbits(ifp, 0, (IFF_DRV_RUNNING | IFF_DRV_OACTIVE));

	e1000_reset_hw(&adapter->hw);
	if (adapter->hw.mac.type >= e1000_82544)
		E1000_WRITE_REG(&adapter->hw, E1000_WUC, 0);

	e1000_led_off(&adapter->hw);
	e1000_cleanup_led(&adapter->hw);
}

/*
 * Bit of a misnomer, what this really means is
 * to enable OS management of the system... aka
 * to disable special hardware management features 
 */
static void
lem_init_manageability(struct adapter *adapter)
{
	/* A shared code workaround */
	if (adapter->has_manage) {
		int manc = E1000_READ_REG(&adapter->hw, E1000_MANC);
		/* disable hardware interception of ARP */
		manc &= ~(E1000_MANC_ARP_EN);
		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}

/*********************************************************************
 *
 *  (Re)Initialize transmit structures.
 *
 **********************************************************************/
static void
lem_setup_transmit_structures(struct adapter *adapter)
{
  E1000 *e1000 = adapter->dev->parent;
  
	struct em_buffer *tx_buffer;
// #ifdef DEV_NETMAP
// 	/* we are already locked */
// 	struct netmap_adapter *na = netmap_getna(adapter->ifp);
// 	struct netmap_slot *slot = netmap_reset(na, NR_TX, 0, 0);
// #endif /* DEV_NETMAP */

	/* Clear the old ring contents */
	bzero(adapter->tx_desc_base,
	    (sizeof(struct e1000_tx_desc)) * adapter->num_tx_desc);

	// /* Free any existing TX buffers */
	// for (int i = 0; i < adapter->num_tx_desc; i++, tx_buffer++) {
	// 	tx_buffer = &adapter->tx_buffer_area[i];
	// 	bus_dmamap_sync(adapter->txtag, tx_buffer->map,
	// 	    BUS_DMASYNC_POSTWRITE);
	// 	bus_dmamap_unload(adapter->txtag, tx_buffer->map);
        //         m_freem(tx_buffer->m_head);
	// 	tx_buffer->m_head = NULL;
        // #ifdef DEV_NETMAP
        // 		if (slot) {
        // 			/* the i-th NIC entry goes to slot si */
        // 			int si = netmap_idx_n2k(&na->tx_rings[0], i);
        // 			uint64_t paddr;
        // 			void *addr;

        // 			addr = PNMB(na, slot + si, &paddr);
        // 			adapter->tx_desc_base[i].buffer_addr = htole64(paddr);
        // 			/* reload the map for netmap mode */
        // 			netmap_load_map(na, adapter->txtag, tx_buffer->map, addr);
        // 		}
        // #endif /* DEV_NETMAP */
	// 	tx_buffer->next_eop = -1;
	// }
        E1000::Packet *packet;
        while(e1000->_tx_buffered.Pop(packet)) {
          kassert(e1000->_tx_reserved.Push(packet));
        }

	for (int i = 0; i < adapter->num_tx_desc; i++) {
          PhysAddr paddr;
          physmem_ctrl->Alloc(paddr, PagingCtrl::ConvertNumToPageSize(MCLBYTES));
          adapter->tx_desc_base[i].buffer_addr = htole64(paddr.GetAddr());
        }

	/* Reset state */
	adapter->last_hw_offload = 0;
	adapter->next_avail_tx_desc = 0;
	adapter->next_tx_to_clean = 0;
	adapter->num_tx_desc_avail = adapter->num_tx_desc;

	bus_dmamap_sync(adapter->txdma.dma_tag, adapter->txdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	return;
}

/*********************************************************************
 *
 *  Enable transmit unit.
 *
 **********************************************************************/
static void
lem_initialize_transmit_unit(struct adapter *adapter)
{
	u32	tctl, tipg = 0;
	u64	bus_addr;

	 INIT_DEBUGOUT("lem_initialize_transmit_unit: begin");
	/* Setup the Base and Length of the Tx Descriptor Ring */
         bus_addr = reinterpret_cast<u64>(adapter->txdma.dma_paddr);
	E1000_WRITE_REG(&adapter->hw, E1000_TDLEN(0),
	    adapter->num_tx_desc * sizeof(struct e1000_tx_desc));
	E1000_WRITE_REG(&adapter->hw, E1000_TDBAH(0),
	    (u32)(bus_addr >> 32));
	E1000_WRITE_REG(&adapter->hw, E1000_TDBAL(0),
	    (u32)bus_addr);
	/* Setup the HW Tx Head and Tail descriptor pointers */
	E1000_WRITE_REG(&adapter->hw, E1000_TDT(0), 0);
	E1000_WRITE_REG(&adapter->hw, E1000_TDH(0), 0);

	HW_DEBUGOUT2("Base = %x, Length = %x\n",
	    E1000_READ_REG(&adapter->hw, E1000_TDBAL(0)),
	    E1000_READ_REG(&adapter->hw, E1000_TDLEN(0)));

	/* Set the default values for the Tx Inter Packet Gap timer */
	switch (adapter->hw.mac.type) {
	case e1000_82542:
		tipg = DEFAULT_82542_TIPG_IPGT;
		tipg |= DEFAULT_82542_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
		tipg |= DEFAULT_82542_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
		break;
	default:
		if ((adapter->hw.phy.media_type == e1000_media_type_fiber) ||
		    (adapter->hw.phy.media_type ==
		    e1000_media_type_internal_serdes))
			tipg = DEFAULT_82543_TIPG_IPGT_FIBER;
		else
			tipg = DEFAULT_82543_TIPG_IPGT_COPPER;
		tipg |= DEFAULT_82543_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
		tipg |= DEFAULT_82543_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
	}

	E1000_WRITE_REG(&adapter->hw, E1000_TIPG, tipg);
	E1000_WRITE_REG(&adapter->hw, E1000_TIDV, adapter->tx_int_delay.value);
	if(adapter->hw.mac.type >= e1000_82540)
		E1000_WRITE_REG(&adapter->hw, E1000_TADV,
		    adapter->tx_abs_int_delay.value);

	/* Program the Transmit Control Register */
	tctl = E1000_READ_REG(&adapter->hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= (E1000_TCTL_PSP | E1000_TCTL_RTLC | E1000_TCTL_EN |
		   (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT));

	/* This write will effectively turn on the transmit unit. */
	E1000_WRITE_REG(&adapter->hw, E1000_TCTL, tctl);

	/* Setup Transmit Descriptor Base Settings */   
	adapter->txd_cmd = E1000_TXD_CMD_IFCS;

	if (adapter->tx_int_delay.value > 0)
		adapter->txd_cmd |= E1000_TXD_CMD_IDE;
}

/*********************************************************************
 *  Multicast Update
 *
 *  This routine is called whenever multicast address list is updated.
 *
 **********************************************************************/

static void
lem_set_multi(struct adapter *adapter)
{
	if_t ifp = adapter->ifp;
	u32 reg_rctl = 0;
	u8  *mta; /* Multicast array memory */
	int mcnt = 0;

	IOCTL_DEBUGOUT("lem_set_multi: begin");

	mta = adapter->mta;
	bzero(mta, sizeof(u8) * ETH_ADDR_LEN * MAX_NUM_MULTICAST_ADDRESSES);

	if (adapter->hw.mac.type == e1000_82542 && 
	    adapter->hw.revision_id == E1000_REVISION_2) {
		reg_rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
		if (adapter->hw.bus.pci_cmd_word & CMD_MEM_WRT_INVALIDATE)
			e1000_pci_clear_mwi(&adapter->hw);
		reg_rctl |= E1000_RCTL_RST;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
		msec_delay(5);
	}

	// if_multiaddr_array(ifp, mta, &mcnt, MAX_NUM_MULTICAST_ADDRESSES);

	if (mcnt >= MAX_NUM_MULTICAST_ADDRESSES) {
		reg_rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
		reg_rctl |= E1000_RCTL_MPE;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
	} else
		e1000_update_mc_addr_list(&adapter->hw, mta, mcnt);

	if (adapter->hw.mac.type == e1000_82542 && 
	    adapter->hw.revision_id == E1000_REVISION_2) {
		reg_rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
		reg_rctl &= ~E1000_RCTL_RST;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
		msec_delay(5);
		if (adapter->hw.bus.pci_cmd_word & CMD_MEM_WRT_INVALIDATE)
			e1000_pci_set_mwi(&adapter->hw);
	}
}

/*********************************************************************
 *
 *  (Re)initialize receive structures.
 *
 **********************************************************************/
static int
lem_setup_receive_structures(struct adapter *adapter)
{
  E1000 *e1000 = adapter->dev->parent;

	struct em_buffer *rx_buffer;
	int i, error;
// #ifdef DEV_NETMAP
// 	/* we are already under lock */
// 	struct netmap_adapter *na = netmap_getna(adapter->ifp);
// 	struct netmap_slot *slot = netmap_reset(na, NR_RX, 0, 0);
// #endif

	/* Reset descriptor ring */
	bzero(adapter->rx_desc_base,
	    (sizeof(struct e1000_rx_desc)) * adapter->num_rx_desc);

	/* Free current RX buffers. */
	// rx_buffer = adapter->rx_buffer_area;
	// for (i = 0; i < adapter->num_rx_desc; i++, rx_buffer++) {
	// 	if (rx_buffer->m_head != NULL) {
	// 		bus_dmamap_sync(adapter->rxtag, rx_buffer->map,
	// 		    BUS_DMASYNC_POSTREAD);
	// 		bus_dmamap_unload(adapter->rxtag, rx_buffer->map);
	// 		m_freem(rx_buffer->m_head);
	// 		rx_buffer->m_head = NULL;
	// 	}
        // }
        E1000::Packet *packet;
        while(e1000->_rx_buffered.Pop(packet)) {
          kassert(e1000->_rx_reserved.Push(packet));
        }

	/* Allocate new ones. */
	for (i = 0; i < adapter->num_rx_desc; i++) {
// #ifdef DEV_NETMAP
// 		if (slot) {
// 			/* the i-th NIC entry goes to slot si */
// 			int si = netmap_idx_n2k(&na->rx_rings[0], i);
// 			uint64_t paddr;
// 			void *addr;

// 			addr = PNMB(na, slot + si, &paddr);
// 			netmap_load_map(na, adapter->rxtag, rx_buffer->map, addr);
// 			/* Update descriptor */
// 			adapter->rx_desc_base[i].buffer_addr = htole64(paddr);
// 			continue;
// 		}
// #endif /* DEV_NETMAP */
		// error = lem_get_buf(adapter, i);
		// if (error)
                //         return (error);
          PhysAddr paddr;
          physmem_ctrl->Alloc(paddr, PagingCtrl::ConvertNumToPageSize(MCLBYTES));
          adapter->rx_desc_base[i].buffer_addr = htole64(paddr.GetAddr());
	}

	/* Setup our descriptor pointers */
	adapter->next_rx_desc_to_check = 0;
	bus_dmamap_sync(adapter->rxdma.dma_tag, adapter->rxdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	return (0);
}

/*********************************************************************
 *
 *  Enable receive unit.
 *
 **********************************************************************/

static void
lem_initialize_receive_unit(struct adapter *adapter)
{
	if_t ifp = adapter->ifp;
	u64	bus_addr;
	u32	rctl, rxcsum;

	INIT_DEBUGOUT("lem_initialize_receive_unit: begin");

	/*
	 * Make sure receives are disabled while setting
	 * up the descriptor ring
	 */
	rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);

	if (adapter->hw.mac.type >= e1000_82540) {
		E1000_WRITE_REG(&adapter->hw, E1000_RADV,
		    adapter->rx_abs_int_delay.value);
		/*
		 * Set the interrupt throttling rate. Value is calculated
		 * as DEFAULT_ITR = 1/(MAX_INTS_PER_SEC * 256ns)
		 */
		E1000_WRITE_REG(&adapter->hw, E1000_ITR, DEFAULT_ITR);
	}

	/* Setup the Base and Length of the Rx Descriptor Ring */
	bus_addr = reinterpret_cast<u64>(adapter->rxdma.dma_paddr);
	E1000_WRITE_REG(&adapter->hw, E1000_RDLEN(0),
	    adapter->num_rx_desc * sizeof(struct e1000_rx_desc));
	E1000_WRITE_REG(&adapter->hw, E1000_RDBAH(0),
	    (u32)(bus_addr >> 32));
	E1000_WRITE_REG(&adapter->hw, E1000_RDBAL(0),
	    (u32)bus_addr);

	/* Setup the Receive Control Register */
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_LBM_NO |
		   E1000_RCTL_RDMTS_HALF |
		   (adapter->hw.mac.mc_filter_type << E1000_RCTL_MO_SHIFT);

	/* Make sure VLAN Filters are off */
	rctl &= ~E1000_RCTL_VFE;

	if (e1000_tbi_sbp_enabled_82543(&adapter->hw))
		rctl |= E1000_RCTL_SBP;
	else
		rctl &= ~E1000_RCTL_SBP;

	switch (adapter->rx_buffer_len) {
	default:
	case 2048:
		rctl |= E1000_RCTL_SZ_2048;
		break;
	case 4096:
		rctl |= E1000_RCTL_SZ_4096 |
		    E1000_RCTL_BSEX | E1000_RCTL_LPE;
		break;
	case 8192:
		rctl |= E1000_RCTL_SZ_8192 |
		    E1000_RCTL_BSEX | E1000_RCTL_LPE;
		break;
	case 16384:
		rctl |= E1000_RCTL_SZ_16384 |
		    E1000_RCTL_BSEX | E1000_RCTL_LPE;
		break;
	}

	if (if_getmtu(ifp) > ETHERMTU)
		rctl |= E1000_RCTL_LPE;
	else
		rctl &= ~E1000_RCTL_LPE;

	/* Enable 82543 Receive Checksum Offload for TCP and UDP */
	// if ((adapter->hw.mac.type >= e1000_82543) &&
	//     (if_getcapenable(ifp) & IFCAP_RXCSUM)) {
	// 	rxcsum = E1000_READ_REG(&adapter->hw, E1000_RXCSUM);
	// 	rxcsum |= (E1000_RXCSUM_IPOFL | E1000_RXCSUM_TUOFL);
	// 	E1000_WRITE_REG(&adapter->hw, E1000_RXCSUM, rxcsum);
	// }

	/* Enable Receives */
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);

	/*
	 * Setup the HW Rx Head and
	 * Tail Descriptor Pointers
	 */
	E1000_WRITE_REG(&adapter->hw, E1000_RDH(0), 0);
	rctl = adapter->num_rx_desc - 1; /* default RDT value */
// #ifdef DEV_NETMAP
// 	/* preserve buffers already made available to clients */
// 	if (if_getcapenable(ifp) & IFCAP_NETMAP) {
// 		struct netmap_adapter *na = netmap_getna(adapter->ifp);
// 		rctl -= nm_kr_rxspace(&na->rx_rings[0]);
// 	}
// #endif /* DEV_NETMAP */
	E1000_WRITE_REG(&adapter->hw, E1000_RDT(0), rctl);

	return;
}

/*********************************************************************
 *  Timer routine
 *
 *  This routine checks for link status and updates statistics.
 *
 **********************************************************************/

static void
lem_local_timer(void *arg)
{
  struct adapter	*adapter = reinterpret_cast<struct adapter *>(arg);

	EM_CORE_LOCK_ASSERT(adapter);

	lem_update_link_status(adapter);
	lem_update_stats_counters(adapter);

	lem_smartspeed(adapter);

// #ifdef NIC_PARAVIRT
// 	/* recover space if needed */
// 	if (adapter->csb && adapter->csb->guest_csb_on &&
// 	    (adapter->watchdog_check == TRUE) &&
// 	    (ticks - adapter->watchdog_time > EM_WATCHDOG) &&
// 	    (adapter->num_tx_desc_avail != adapter->num_tx_desc) ) {
// 		lem_txeof(adapter);
// 		/*
// 		 * lem_txeof() normally (except when space in the queue
// 		 * runs low XXX) cleans watchdog_check so that
// 		 * we do not hung.
// 		 */
// 	}
// #endif /* NIC_PARAVIRT */
	/*
	 * We check the watchdog: the time since
	 * the last TX descriptor was cleaned.
	 * This implies a functional TX engine.
	 */
	if ((adapter->watchdog_check == TRUE) &&
	    (get_ticks() - adapter->watchdog_time > EM_WATCHDOG))
		goto hung;

	callout_reset(&adapter->timer, hz, lem_local_timer, adapter);
	return;
hung:
	device_printf(adapter->dev, "Watchdog timeout -- resetting\n");
	if_setdrvflagbits(adapter->ifp, 0, IFF_DRV_RUNNING);
	adapter->watchdog_events++;
	lem_init_locked(adapter);
}

/*********************************************************************
 *
 *  Workaround for SmartSpeed on 82541 and 82547 controllers
 *
 **********************************************************************/
static void
lem_smartspeed(struct adapter *adapter)
{
	u16 phy_tmp;

	if (adapter->link_active || (adapter->hw.phy.type != e1000_phy_igp) ||
	    adapter->hw.mac.autoneg == 0 ||
	    (adapter->hw.phy.autoneg_advertised & ADVERTISE_1000_FULL) == 0)
		return;

	if (adapter->smartspeed == 0) {
		/* If Master/Slave config fault is asserted twice,
		 * we assume back-to-back */
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_STATUS, &phy_tmp);
		if (!(phy_tmp & SR_1000T_MS_CONFIG_FAULT))
			return;
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_STATUS, &phy_tmp);
		if (phy_tmp & SR_1000T_MS_CONFIG_FAULT) {
			e1000_read_phy_reg(&adapter->hw,
			    PHY_1000T_CTRL, &phy_tmp);
			if(phy_tmp & CR_1000T_MS_ENABLE) {
				phy_tmp &= ~CR_1000T_MS_ENABLE;
				e1000_write_phy_reg(&adapter->hw,
				    PHY_1000T_CTRL, phy_tmp);
				adapter->smartspeed++;
				if(adapter->hw.mac.autoneg &&
				   !e1000_copper_link_autoneg(&adapter->hw) &&
				   !e1000_read_phy_reg(&adapter->hw,
				    PHY_CONTROL, &phy_tmp)) {
					phy_tmp |= (MII_CR_AUTO_NEG_EN |
						    MII_CR_RESTART_AUTO_NEG);
					e1000_write_phy_reg(&adapter->hw,
					    PHY_CONTROL, phy_tmp);
				}
			}
		}
		return;
	} else if(adapter->smartspeed == EM_SMARTSPEED_DOWNSHIFT) {
		/* If still no link, perhaps using 2/3 pair cable */
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_CTRL, &phy_tmp);
		phy_tmp |= CR_1000T_MS_ENABLE;
		e1000_write_phy_reg(&adapter->hw, PHY_1000T_CTRL, phy_tmp);
		if(adapter->hw.mac.autoneg &&
		   !e1000_copper_link_autoneg(&adapter->hw) &&
		   !e1000_read_phy_reg(&adapter->hw, PHY_CONTROL, &phy_tmp)) {
			phy_tmp |= (MII_CR_AUTO_NEG_EN |
				    MII_CR_RESTART_AUTO_NEG);
			e1000_write_phy_reg(&adapter->hw, PHY_CONTROL, phy_tmp);
		}
	}
	/* Restart process after EM_SMARTSPEED_MAX iterations */
	if(adapter->smartspeed++ == EM_SMARTSPEED_MAX)
		adapter->smartspeed = 0;
}

/*********************************************************************
 *
 *  This routine executes in interrupt context. It replenishes
 *  the mbufs in the descriptor and sends data which has been
 *  dma'ed into host memory to upper layer.
 *
 *  We loop at most count times if count is > 0, or until done if
 *  count < 0.
 *  
 *  For polling we also now return the number of cleaned packets
 *********************************************************************/
static bool
lem_rxeof(struct adapter *adapter, int count, int *done)
{
  if_t ifp = adapter->ifp;
  struct mbuf	*mp;
  u8		status = 0, accept_frame = 0, eop = 0;
  u16 		len, desc_len, prev_len_adj;
  int		i, rx_sent = 0;
  struct e1000_rx_desc   *current_desc;
  E1000 *e1000 = adapter->dev->parent;

  // #ifdef BATCH_DISPATCH
  // 	struct mbuf *mh = NULL, *mt = NULL;
  // #endif /* BATCH_DISPATCH */
  // #ifdef NIC_PARAVIRT
  // 	int retries = 0;
  // 	struct paravirt_csb* csb = adapter->csb;
  // 	int csb_mode = csb && csb->guest_csb_on;

  // 	//ND("clear guest_rxkick at %d", adapter->next_rx_desc_to_check);
  // 	if (csb_mode && csb->guest_need_rxkick)
  // 		csb->guest_need_rxkick = 0;
  // #endif /* NIC_PARAVIRT */
  EM_RX_LOCK(adapter);

  // #ifdef BATCH_DISPATCH
  //     batch_again:
  // #endif /* BATCH_DISPATCH */
  i = adapter->next_rx_desc_to_check;
  current_desc = &adapter->rx_desc_base[i];
  bus_dmamap_sync(adapter->rxdma.dma_tag, adapter->rxdma.dma_map,
                  BUS_DMASYNC_POSTREAD);
    
  // #ifdef DEV_NETMAP
  // 	if (netmap_rx_irq(ifp, 0, &rx_sent)) {
  // 		EM_RX_UNLOCK(adapter);
  // 		return (FALSE);
  // 	}
  // #endif /* DEV_NETMAP */

  // #if 1 // XXX optimization ?
  if (!((current_desc->status) & E1000_RXD_STAT_DD)) {
    if (done != NULL)
      *done = rx_sent;
    EM_RX_UNLOCK(adapter);
    return (FALSE);
  }
  // #endif /* 0 */
  
  while (count != 0 && if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
    struct mbuf *m = NULL;
  
    status = current_desc->status;
    if ((status & E1000_RXD_STAT_DD) == 0) {
      // #ifdef NIC_PARAVIRT
      // 		    if (csb_mode) {
      // 			/* buffer not ready yet. Retry a few times before giving up */
      // 			if (++retries <= adapter->rx_retries) {
      // 				continue;
      // 			}
      // 			if (csb->guest_need_rxkick == 0) {
      // 				// ND("set guest_rxkick at %d", adapter->next_rx_desc_to_check);
      // 				csb->guest_need_rxkick = 1;
      // 				// XXX memory barrier, status volatile ?
      // 				continue; /* double check */
      // 			}
      // 		    }
      // 		    /* no buffer ready, give up */
      // #endif /* NIC_PARAVIRT */
      break;
    }
    // #ifdef NIC_PARAVIRT
    // 		if (csb_mode) {
    // 			if (csb->guest_need_rxkick)
    // 				// ND("clear again guest_rxkick at %d", adapter->next_rx_desc_to_check);
    // 			csb->guest_need_rxkick = 0;
    // 			retries = 0;
    // 		}
    // #endif /* NIC_PARAVIRT */

    mp = adapter->rx_buffer_area[i].m_head;
    /*
     * Can't defer bus_dmamap_sync(9) because TBI_ACCEPT
     * needs to access the last received byte in the mbuf.
     */
    bus_dmamap_sync(adapter->rxtag, adapter->rx_buffer_area[i].map,
                    BUS_DMASYNC_POSTREAD);

    accept_frame = 1;
    prev_len_adj = 0;
    desc_len = le16toh(current_desc->length);
    if (status & E1000_RXD_STAT_EOP) {
      count--;
      eop = 1;
      if (desc_len < ETHER_CRC_LEN) {
        len = 0;
        prev_len_adj = ETHER_CRC_LEN - desc_len;
      } else
        len = desc_len - ETHER_CRC_LEN;
    } else {
      eop = 0;
      len = desc_len;
    }

    if (current_desc->errors & E1000_RXD_ERR_FRAME_ERR_MASK) {
      u8	last_byte;
      u32	pkt_len = desc_len;

      // if (adapter->fmp != NULL)
      //   pkt_len += adapter->fmp->m_pkthdr.len;

      // last_byte = *(mtod(mp, caddr_t) + desc_len - 1);			
      last_byte = *(reinterpret_cast<uint8_t *>(p2v(adapter->rx_desc_base[i].buffer_addr)) + desc_len - 1);			
      if (TBI_ACCEPT(&adapter->hw, status,
                     current_desc->errors, pkt_len, last_byte,
                     adapter->min_frame_size, adapter->max_frame_size)) {
        e1000_tbi_adjust_stats_82543(&adapter->hw,
                                     &adapter->stats, pkt_len,
                                     adapter->hw.mac.addr,
                                     adapter->max_frame_size);
        if (len > 0)
          len--;
      } else
        accept_frame = 0;
    }

    if (accept_frame) {
      E1000::Packet *packet;
      if (e1000->_rx_reserved.Pop(packet)) {
        memcpy(packet->buf, reinterpret_cast<void *>(p2v(adapter->rx_desc_base[i].buffer_addr)), len);
        packet->len = len;
        if (!e1000->_rx_buffered.Push(packet)) {
          kassert(e1000->_rx_reserved.Push(packet));
          adapter->dropped_pkts++;
        }
      } else {
        adapter->dropped_pkts++;
      }
      // 			if (lem_get_buf(adapter, i) != 0) {
      // 				// if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
      // 				goto discard;
      // 			}

      // 			/* Assign correct length to the current fragment */
      // 			mp->m_len = len;

      // 			if (adapter->fmp == NULL) {
      // 				mp->m_pkthdr.len = len;
      // 				adapter->fmp = mp; /* Store the first mbuf */
      // 				adapter->lmp = mp;
      // 			} else {
      // 				/* Chain mbuf's together */
      // 				mp->m_flags &= ~M_PKTHDR;
      // 				/*
      // 				 * Adjust length of previous mbuf in chain if
      // 				 * we received less than 4 bytes in the last
      // 				 * descriptor.
      // 				 */
      // 				if (prev_len_adj > 0) {
      // 					adapter->lmp->m_len -= prev_len_adj;
      // 					adapter->fmp->m_pkthdr.len -=
      // 					    prev_len_adj;
      // 				}
      // 				adapter->lmp->m_next = mp;
      // 				adapter->lmp = adapter->lmp->m_next;
      // 				adapter->fmp->m_pkthdr.len += len;
      // 			}

      // 			if (eop) {
      // 				// if_setrcvif(adapter->fmp, ifp);
      // 				// if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
      // 				lem_receive_checksum(adapter, current_desc,
      // 				    adapter->fmp);
      // #ifndef __NO_STRICT_ALIGNMENT
      // 				if (adapter->max_frame_size >
      // 				    (MCLBYTES - ETHER_ALIGN) &&
      // 				    lem_fixup_rx(adapter) != 0)
      // 					goto skip;
      // #endif
      // 				if (status & E1000_RXD_STAT_VP) {
      // 					adapter->fmp->m_pkthdr.ether_vtag =
      // 					    le16toh(current_desc->special);
      // 					adapter->fmp->m_flags |= M_VLANTAG;
      // 				}
      // #ifndef __NO_STRICT_ALIGNMENT
      // skip:
      // #endif
      // 				m = adapter->fmp;
      // 				adapter->fmp = NULL;
      // 				adapter->lmp = NULL;
      // 			}
    } else {
      adapter->dropped_pkts++;
      // discard:
      // /* Reuse loaded DMA map and just update mbuf chain */
      // mp = adapter->rx_buffer_area[i].m_head;
      // mp->m_len = mp->m_pkthdr.len = MCLBYTES;
      // mp->m_data = mp->m_ext.ext_buf;
      // mp->m_next = NULL;
      // if (adapter->max_frame_size <=
      //     (MCLBYTES - ETHER_ALIGN))
      // 	m_adj(mp, ETHER_ALIGN);
      // if (adapter->fmp != NULL) {
      // 	m_freem(adapter->fmp);
      // 	adapter->fmp = NULL;
      // 	adapter->lmp = NULL;
      // }
      // m = NULL;
    }

    /* Zero out the receive descriptors status. */
    current_desc->status = 0;
    bus_dmamap_sync(adapter->rxdma.dma_tag, adapter->rxdma.dma_map,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

    // #ifdef NIC_PARAVIRT
    // 		if (csb_mode) {
    // 			/* the buffer at i has been already replaced by lem_get_buf()
    // 			 * so it is safe to set guest_rdt = i and possibly send a kick.
    // 			 * XXX see if we can optimize it later.
    // 			 */
    // 			csb->guest_rdt = i;
    // 			// XXX memory barrier
    // 			if (i == csb->host_rxkick_at)
    // 				E1000_WRITE_REG(&adapter->hw, E1000_RDT(0), i);
    // 		}
    // #endif /* NIC_PARAVIRT */
    /* Advance our pointers to the next descriptor. */
    if (++i == adapter->num_rx_desc)
      i = 0;
    /* Call into the stack */
    if (m != NULL) {
      // #ifdef BATCH_DISPATCH
      // 		    if (adapter->batch_enable) {
      // 			if (mh == NULL)
      // 				mh = mt = m;
      // 			else
      // 				mt->m_nextpkt = m;
      // 			mt = m;
      // 			m->m_nextpkt = NULL;
      // 			rx_sent++;
      // 			current_desc = &adapter->rx_desc_base[i];
      // 			continue;
      // 		    }
      // #endif /* BATCH_DISPATCH */
      adapter->next_rx_desc_to_check = i;
      EM_RX_UNLOCK(adapter);
      // if_input(ifp, m);
      EM_RX_LOCK(adapter);
      rx_sent++;
      i = adapter->next_rx_desc_to_check;
    }
    current_desc = &adapter->rx_desc_base[i];
  }
  adapter->next_rx_desc_to_check = i;
  // #ifdef BATCH_DISPATCH
  // 	if (mh) {
  // 		EM_RX_UNLOCK(adapter);
  // 		while ( (mt = mh) != NULL) {
  // 			mh = mh->m_nextpkt;
  // 			mt->m_nextpkt = NULL;
  // 			if_input(ifp, mt);
  // 		}
  // 		EM_RX_LOCK(adapter);
  // 		i = adapter->next_rx_desc_to_check; /* in case of interrupts */
  // 		if (count > 0)
  // 			goto batch_again;
  // 	}
  // #endif /* BATCH_DISPATCH */

  /* Advance the E1000's Receive Queue #0  "Tail Pointer". */
  if (--i < 0)
    i = adapter->num_rx_desc - 1;
  // #ifdef NIC_PARAVIRT
  // 	if (!csb_mode) /* filter out writes */
  // #endif /* NIC_PARAVIRT */
  E1000_WRITE_REG(&adapter->hw, E1000_RDT(0), i);
  if (done != NULL)
    *done = rx_sent;
  EM_RX_UNLOCK(adapter);
  return ((status & E1000_RXD_STAT_DD) ? TRUE : FALSE);
}

/**********************************************************************
 *
 *  Examine each tx_buffer in the used queue. If the hardware is done
 *  processing the packet then free associated resources. The
 *  tx_buffer is put back on the free queue.
 *
 **********************************************************************/
static void
lem_txeof(struct adapter *adapter)
{
        int first, last, done, num_avail;
        struct em_buffer *tx_buffer;
        struct e1000_tx_desc   *tx_desc, *eop_desc;
	if_t ifp = adapter->ifp;

	EM_TX_LOCK_ASSERT(adapter);

// #ifdef DEV_NETMAP
// 	if (netmap_tx_irq(ifp, 0))
// 		return;
// #endif /* DEV_NETMAP */
        if (adapter->num_tx_desc_avail == adapter->num_tx_desc)
                return;

        num_avail = adapter->num_tx_desc_avail;
        first = adapter->next_tx_to_clean;
        tx_desc = &adapter->tx_desc_base[first];
        tx_buffer = &adapter->tx_buffer_area[first];
	last = tx_buffer->next_eop;
        eop_desc = &adapter->tx_desc_base[last];

	/*
	 * What this does is get the index of the
	 * first descriptor AFTER the EOP of the 
	 * first packet, that way we can do the
	 * simple comparison on the inner while loop.
	 */
	if (++last == adapter->num_tx_desc)
 		last = 0;
	done = last;

        bus_dmamap_sync(adapter->txdma.dma_tag, adapter->txdma.dma_map,
            BUS_DMASYNC_POSTREAD);

        while (eop_desc->upper.fields.status & E1000_TXD_STAT_DD) {
		/* We clean the range of the packet */
		while (first != done) {
                	tx_desc->upper.data = 0;
                	tx_desc->lower.data = 0;
                	tx_desc->buffer_addr = 0;
                	++num_avail;

			if (tx_buffer->m_head) {
				// if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
				bus_dmamap_sync(adapter->txtag,
				    tx_buffer->map,
				    BUS_DMASYNC_POSTWRITE);
				bus_dmamap_unload(adapter->txtag,
				    tx_buffer->map);

                        	// m_freem(tx_buffer->m_head);
                        	tx_buffer->m_head = NULL;
                	}
			tx_buffer->next_eop = -1;
			adapter->watchdog_time = get_ticks();

	                if (++first == adapter->num_tx_desc)
				first = 0;

	                tx_buffer = &adapter->tx_buffer_area[first];
			tx_desc = &adapter->tx_desc_base[first];
		}
		/* See if we can continue to the next packet */
		last = tx_buffer->next_eop;
		if (last != -1) {
        		eop_desc = &adapter->tx_desc_base[last];
			/* Get new done point */
			if (++last == adapter->num_tx_desc) last = 0;
			done = last;
		} else
			break;
        }
        bus_dmamap_sync(adapter->txdma.dma_tag, adapter->txdma.dma_map,
            BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

        adapter->next_tx_to_clean = first;
        adapter->num_tx_desc_avail = num_avail;

// #ifdef NIC_SEND_COMBINING
// 	if ((adapter->shadow_tdt & MIT_PENDING_TDT) == MIT_PENDING_TDT) {
// 		/* a tdt write is pending, do it */
// 		E1000_WRITE_REG(&adapter->hw, E1000_TDT(0),
// 			0xffff & adapter->shadow_tdt);
// 		adapter->shadow_tdt = MIT_PENDING_INT;
// 	} else {
// 		adapter->shadow_tdt = 0; // disable
// 	}
// #endif /* NIC_SEND_COMBINING */
        /*
         * If we have enough room, clear IFF_DRV_OACTIVE to
         * tell the stack that it is OK to send packets.
         * If there are no pending descriptors, clear the watchdog.
         */
        if (adapter->num_tx_desc_avail > EM_TX_CLEANUP_THRESHOLD) {                
                if_setdrvflagbits(ifp, 0, IFF_DRV_OACTIVE);
// #ifdef NIC_PARAVIRT
// 		if (adapter->csb) { // XXX also csb_on ?
// 			adapter->csb->guest_need_txkick = 2; /* acked */
// 			// XXX memory barrier
// 		}
// #endif /* NIC_PARAVIRT */
                if (adapter->num_tx_desc_avail == adapter->num_tx_desc) {
			adapter->watchdog_check = FALSE;
			return;
		} 
        }
}

static void
lem_start(if_t ifp)
{
  struct adapter *adapter = reinterpret_cast<struct adapter *>(if_getsoftc(ifp));

	EM_TX_LOCK(adapter);
	if (if_getdrvflags(ifp) & IFF_DRV_RUNNING)
		lem_start_locked(ifp);
	EM_TX_UNLOCK(adapter);
}

static void
lem_start_locked(if_t ifp)
{
  struct adapter	*adapter = reinterpret_cast<struct adapter *>(if_getsoftc(ifp));
  E1000 *e1000 = adapter->dev->parent;
	struct mbuf	*m_head;

	EM_TX_LOCK_ASSERT(adapter);

	if ((if_getdrvflags(ifp) & (IFF_DRV_RUNNING|IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING)
		return;
	if (!adapter->link_active)
		return;

        /*
         * Force a cleanup if number of TX descriptors
         * available hits the threshold
         */
	if (adapter->num_tx_desc_avail <= EM_TX_CLEANUP_THRESHOLD) {
		lem_txeof(adapter);
		/* Now do we at least have a minimal? */
		if (adapter->num_tx_desc_avail <= EM_TX_OP_THRESHOLD) {
			adapter->no_tx_desc_avail1++;
			return;
		}
	}

	// while (!if_sendq_empty(ifp)) {
        while (!e1000->_tx_buffered.IsEmpty()) {
          // m_head = if_dequeue(ifp);

	  //       if (m_head == NULL)
	  //       	break;
	  //       /*
	  //        *  Encapsulation can modify our pointer, and or make it
	  //        *  NULL on failure.  In that event, we can't requeue.
	  //        */
	  //       if (lem_xmit(adapter, &m_head)) {
	  //       	if (m_head == NULL)
	  //       		break;
	  //       	if_setdrvflagbits(ifp, IFF_DRV_OACTIVE, 0);
	  //       	if_sendq_prepend(ifp, m_head);
	  //       	break;
	  //       }
          E1000::Packet *packet;
          kassert(e1000->_tx_buffered.Pop(packet));
          lem_xmit(adapter, packet);

		/* Send a copy of the frame to the BPF listener */
		// if_etherbpfmtap(ifp, m_head);

		/* Set timeout in case hardware has problems transmitting. */
		adapter->watchdog_check = TRUE;
		adapter->watchdog_time = get_ticks();
	}
	if (adapter->num_tx_desc_avail <= EM_TX_OP_THRESHOLD)
		if_setdrvflagbits(ifp, IFF_DRV_OACTIVE, 0);
#ifdef NIC_PARAVIRT
	if (if_getdrvflags(ifp) & IFF_DRV_OACTIVE && adapter->csb &&
	    adapter->csb->guest_csb_on &&
	    !(adapter->csb->guest_need_txkick & 1))  {
		adapter->csb->guest_need_txkick = 1;
		adapter->guest_need_kick_count++;
		// XXX memory barrier
		lem_txeof(adapter); // XXX possibly clear IFF_DRV_OACTIVE
	}
#endif /* NIC_PARAVIRT */

	return;
}

/*********************************************************************
 *
 *  This routine maps the mbufs to tx descriptors.
 *
 *  return 0 on success, positive on failure
 **********************************************************************/

static int
lem_xmit(struct adapter *adapter, E1000::Packet *packet)
{
	bus_dma_segment_t	segs[EM_MAX_SCATTER];
	bus_dmamap_t		map;
	struct em_buffer	*tx_buffer, *tx_buffer_mapped;
	struct e1000_tx_desc	*ctxd = NULL;
	struct mbuf		*m_head;
	u32			txd_upper, txd_lower, txd_used, txd_saved;
	int			error, nsegs, i, j, first, last = 0;
        E1000 *e1000 = adapter->dev->parent;

        // m_head = *m_headp;
	txd_upper = txd_lower = txd_used = txd_saved = 0;

	/*
	** When doing checksum offload, it is critical to
	** make sure the first mbuf has more than header,
	** because that routine expects data to be present.
	*/
	// if ((m_head->m_pkthdr.csum_flags & CSUM_OFFLOAD) &&
	//     (m_head->m_len < ETHER_HDR_LEN + sizeof(struct ip))) {
	// 	m_head = m_pullup(m_head, ETHER_HDR_LEN + sizeof(struct ip));
	// 	*m_headp = m_head;
	// 	if (m_head == NULL)
	// 		return (ENOBUFS);
	// }

	/*
	 * Map the packet for DMA
	 *
	 * Capture the first descriptor index,
	 * this descriptor will have the index
	 * of the EOP which is the only one that
	 * now gets a DONE bit writeback.
	 */
	first = adapter->next_avail_tx_desc;
	tx_buffer = &adapter->tx_buffer_area[first];
	tx_buffer_mapped = tx_buffer;
	map = tx_buffer->map;

	// error = bus_dmamap_load_mbuf_sg(adapter->txtag, map,
	//     *m_headp, segs, &nsegs, BUS_DMA_NOWAIT);

	/*
	 * There are two types of errors we can (try) to handle:
	 * - EFBIG means the mbuf chain was too long and bus_dma ran
	 *   out of segments.  Defragment the mbuf chain and try again.
	 * - ENOMEM means bus_dma could not obtain enough bounce buffers
	 *   at this point in time.  Defer sending and try again later.
	 * All other errors, in particular EINVAL, are fatal and prevent the
	 * mbuf chain from ever going through.  Drop it and report error.
	 */
	if (error == EFBIG) {
          kassert(false);
		// struct mbuf *m;

		// m = m_defrag(*m_headp, M_NOWAIT);
		// if (m == NULL) {
		// 	adapter->mbuf_alloc_failed++;
		// 	m_freem(*m_headp);
		// 	*m_headp = NULL;
		// 	return (ENOBUFS);
		// }
		// *m_headp = m;

		/* Try it again */
		// error = bus_dmamap_load_mbuf_sg(adapter->txtag, map,
		//     *m_headp, segs, &nsegs, BUS_DMA_NOWAIT);

		// if (error) {
		// 	adapter->no_tx_dma_setup++;
		// 	m_freem(*m_headp);
		// 	*m_headp = NULL;
		// 	return (error);
		// }
	} else if (error != 0) {
		adapter->no_tx_dma_setup++;
		return (error);
	}

        // if (nsegs > (adapter->num_tx_desc_avail - 2)) {
        //         adapter->no_tx_desc_avail2++;
	// 	bus_dmamap_unload(adapter->txtag, map);
	// 	return (ENOBUFS);
        // }
	// m_head = *m_headp;

	/* Do hardware assists */
	// if (m_head->m_pkthdr.csum_flags & CSUM_OFFLOAD)
		// lem_transmit_checksum_setup(adapter,  m_head,
		//     &txd_upper, &txd_lower);

	i = adapter->next_avail_tx_desc;
	if (adapter->pcix_82544) 
		txd_saved = i;

	/* Set up our transmit descriptors */
	// for (j = 0; j < nsegs; j++) {
	// 	bus_size_t seg_len;
	// 	bus_addr_t seg_addr;
	// 	/* If adapter is 82544 and on PCIX bus */
	// 	if(adapter->pcix_82544) {
	// 		DESC_ARRAY	desc_array;
	// 		u32		array_elements, counter;
	// 		/*
	// 		 * Check the Address and Length combination and
	// 		 * split the data accordingly
	// 		 */
	// 		array_elements = lem_fill_descriptors(segs[j].ds_addr,
	// 		    segs[j].ds_len, &desc_array);
	// 		for (counter = 0; counter < array_elements; counter++) {
	// 			if (txd_used == adapter->num_tx_desc_avail) {
	// 				adapter->next_avail_tx_desc = txd_saved;
	// 				adapter->no_tx_desc_avail2++;
	// 				bus_dmamap_unload(adapter->txtag, map);
	// 				return (ENOBUFS);
	// 			}
	// 			tx_buffer = &adapter->tx_buffer_area[i];
	// 			ctxd = &adapter->tx_desc_base[i];
	// 			ctxd->buffer_addr = htole64(
	// 			    desc_array.descriptor[counter].address);
	// 			ctxd->lower.data = htole32(
	// 			    (adapter->txd_cmd | txd_lower | (u16)
	// 			    desc_array.descriptor[counter].length));
	// 			ctxd->upper.data =
	// 			    htole32((txd_upper));
	// 			last = i;
	// 			if (++i == adapter->num_tx_desc)
        //                                  i = 0;
	// 			tx_buffer->m_head = NULL;
	// 			tx_buffer->next_eop = -1;
	// 			txd_used++;
        //                 }
	// 	} else {
	// 		tx_buffer = &adapter->tx_buffer_area[i];
	// 		ctxd = &adapter->tx_desc_base[i];
	// 		seg_addr = segs[j].ds_addr;
	// 		seg_len  = segs[j].ds_len;
	// 		ctxd->buffer_addr = htole64(seg_addr);
	// 		ctxd->lower.data = htole32(
	// 		adapter->txd_cmd | txd_lower | seg_len);
	// 		ctxd->upper.data =
	// 		    htole32(txd_upper);
	// 		last = i;
	// 		if (++i == adapter->num_tx_desc)
	// 			i = 0;
	// 		tx_buffer->m_head = NULL;
	// 		tx_buffer->next_eop = -1;
	// 	}
	// }

        bus_size_t seg_len;
        bus_addr_t seg_addr;
        tx_buffer = &adapter->tx_buffer_area[i];
        ctxd = &adapter->tx_desc_base[i];
        seg_len = packet->len;
        memcpy(reinterpret_cast<void *>(ctxd->buffer_addr), packet->buf, seg_len);
        e1000->ReuseTxBuffer(packet);
        ctxd->lower.data = htole32(
                                   adapter->txd_cmd | txd_lower | seg_len);
        ctxd->upper.data =
          htole32(txd_upper);
        last = i;
        if (++i == adapter->num_tx_desc)
          i = 0;
        tx_buffer->m_head = NULL;
        tx_buffer->next_eop = -1;
        

	adapter->next_avail_tx_desc = i;

	if (adapter->pcix_82544)
		adapter->num_tx_desc_avail -= txd_used;
	else
		adapter->num_tx_desc_avail -= nsegs;

	// if (m_head->m_flags & M_VLANTAG) {
	// 	/* Set the vlan id. */
	// 	ctxd->upper.fields.special =
	// 	    htole16(m_head->m_pkthdr.ether_vtag);
        //         /* Tell hardware to add tag */
        //         ctxd->lower.data |= htole32(E1000_TXD_CMD_VLE);
        // }

        tx_buffer->m_head = m_head;
	tx_buffer_mapped->map = tx_buffer->map;
	tx_buffer->map = map;
        bus_dmamap_sync(adapter->txtag, map, BUS_DMASYNC_PREWRITE);

        /*
         * Last Descriptor of Packet
	 * needs End Of Packet (EOP)
	 * and Report Status (RS)
         */
        ctxd->lower.data |=
	    htole32(E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);
	/*
	 * Keep track in the first buffer which
	 * descriptor will be written back
	 */
	tx_buffer = &adapter->tx_buffer_area[first];
	tx_buffer->next_eop = last;
	adapter->watchdog_time = get_ticks();

	/*
	 * Advance the Transmit Descriptor Tail (TDT), this tells the E1000
	 * that this frame is available to transmit.
	 */
	bus_dmamap_sync(adapter->txdma.dma_tag, adapter->txdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

// #ifdef NIC_PARAVIRT
// 	if (adapter->csb) {
// 		adapter->csb->guest_tdt = i;
// 		/* XXX memory barrier ? */
//  		if (adapter->csb->guest_csb_on &&
// 		    !(adapter->csb->host_need_txkick & 1)) {
// 			/* XXX maybe useless
// 			 * clean the ring. maybe do it before ?
// 			 * maybe a little bit of histeresys ?
// 			 */
// 			if (adapter->num_tx_desc_avail <= 64) {// XXX
// 				lem_txeof(adapter);
// 			}
// 			return (0);
// 		}
// 	}
// #endif /* NIC_PARAVIRT */

// #ifdef NIC_SEND_COMBINING
// 	if (adapter->sc_enable) {
// 		if (adapter->shadow_tdt & MIT_PENDING_INT) {
// 			/* signal intr and data pending */
// 			adapter->shadow_tdt = MIT_PENDING_TDT | (i & 0xffff);
// 			return (0);
// 		} else {
// 			adapter->shadow_tdt = MIT_PENDING_INT;
// 		}
// 	}
// #endif /* NIC_SEND_COMBINING */

	// if (adapter->hw.mac.type == e1000_82547 &&
	//     adapter->link_duplex == HALF_DUPLEX)
        // lem_82547_move_tail(adapter);
	// else {
		E1000_WRITE_REG(&adapter->hw, E1000_TDT(0), i);
		// if (adapter->hw.mac.type == e1000_82547)
		// 	lem_82547_update_fifo_head(adapter,
		// 	    m_head->m_pkthdr.len);
	// }

	return (0);
}
