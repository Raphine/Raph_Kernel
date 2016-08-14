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

#include <sys/bus.h>
#include <sys/bus_dma.h>
#include <raph.h>
#include <mem/paging.h>

extern "C" {

  void _bus_dmamap_sync(bus_dma_tag_t, bus_dmamap_t, bus_dmasync_op_t) {
  }

  int bus_dmamap_create(bus_dma_tag_t dmat, int flags, bus_dmamap_t *mapp) {
    return 0;
  }
  
  void _bus_dmamap_unload(bus_dma_tag_t dmat, bus_dmamap_t map) {
  }

  int bus_dma_tag_create(bus_dma_tag_t parent, bus_size_t alignment,
                         bus_addr_t boundary, bus_addr_t lowaddr,
                         bus_addr_t highaddr, bus_dma_filter_t *filtfunc,
                         void *filtfuncarg, bus_size_t maxsize, int nsegments,
                         bus_size_t maxsegsz, int flags, bus_dma_lock_t *lockfunc,
                         void *lockfuncarg, bus_dma_tag_t *dmat) {
    *dmat = new bus_dma_tag;
    (*dmat)->size = PagingCtrl::RoundUpAddrOnPageBoundary(maxsize);
    return 0;
  }

  int bus_dmamem_alloc(bus_dma_tag_t dmat, void** vaddr, int flags,
                       bus_dmamap_t *mapp) {
    physmem_ctrl->Alloc(dmat->paddr, dmat->size);
    *vaddr = reinterpret_cast<void *>(p2v(dmat->paddr.GetAddr()));
    return 0;
  }

  void bus_dmamem_free(bus_dma_tag_t dmat, void *vaddr, bus_dmamap_t map) {
    physmem_ctrl->Free(dmat->paddr, dmat->size);
  }

  int bus_dmamap_load(bus_dma_tag_t dmat, bus_dmamap_t map, void *buf,
		    bus_size_t buflen, bus_dmamap_callback_t *callback,
                      void *callback_arg, int flags) {
    bus_dma_segment_t segs[0];
    segs[0].ds_addr = reinterpret_cast<bus_addr_t>(dmat->paddr.GetAddr());
    segs[0].ds_len = dmat->size;
    callback(callback_arg, segs,1, 0);
    return 0;
  }

  bus_dma_tag_t bus_get_dma_tag(device_t dev) {
    return nullptr;
  }

  int bus_dma_tag_destroy(bus_dma_tag_t dmat) {
    return 0;
  }
}
