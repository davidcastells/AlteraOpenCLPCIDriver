/* 
 * Copyright (c) 2014, Altera Corporation.
 * All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * BSD 3-Clause license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 *      - Neither Altera nor the names of its contributors may be 
 *        used to endorse or promote products derived from this 
 *        software without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Defines used only by aclpci_dma.c. */


#if USE_DMA

/* Enable Linux-specific defines in the hw_pcie_dma.h file */
#define LINUX
#include <linux/workqueue.h>
#include "hw_pcie_dma.h"
#include "aclpci_queue.h"

struct dma_t {
  void *ptr;         /* if ptr is NULL, the whole struct considered invalid */
  size_t len;
  enum dma_data_direction dir;
  struct page **pages;     /* one for each struct page */
  dma_addr_t *dma_addrs;   /* one for each struct page */
  unsigned int num_pages;
};

struct pinned_mem {
  struct dma_t dma;
  struct page **next_page;
  dma_addr_t *next_dma_addr;
  unsigned int pages_rem;
  unsigned int first_page_offset;
  unsigned int last_page_offset;
};

struct work_struct_t{
   struct work_struct work;
   void *data;
};

struct aclpci_dma {

  // Update information
  unsigned int m_descriptors_updated;
  unsigned int m_descriptors_acknowledged;
  unsigned int m_descriptors_sent;
  unsigned int m_old_done_count;
  size_t m_bytes_acknowledged;

  // The container of pending memory transactions.
  // Contains wd_dma values
  struct queue m_dma_pending;

  // A representation of the hardware's descriptor fifo
  // Contains DESCRIPTOR_UPDATE_DATA values
  struct queue m_desc_pending;

  // Pinned memory we're currently building DMA transactions for.
  struct pinned_mem m_active_mem;

  // The transaction we are currently working on
  struct DMA_DESCRIPTOR m_active_descriptor;

  int m_active_descriptor_valid;
  unsigned int m_active_descriptor_size;
  // The next ATT table row to write to
  unsigned int m_next_att_row;
  // The total number of active ATT rows
  unsigned int m_att_size;

  struct pci_dev *m_pci_dev;
  struct aclpci_dev *m_aclpci;

  // workqueue and work structure for bottom-half interrupt routine
  struct workqueue_struct *my_wq;
  struct work_struct_t *my_work;
  
  // Transfer information
  size_t m_device_addr;
  void* m_host_addr;
  int m_read;
  size_t m_bytes;
  size_t m_bytes_sent;
  int m_idle;

  u64 m_update_time, m_pin_time, m_start_time;
  u64 m_lock_time, m_unlock_time;
};

#else
struct aclpci_dma {};
#endif
