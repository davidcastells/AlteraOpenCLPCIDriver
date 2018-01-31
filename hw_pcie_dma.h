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

#ifndef HW_PCIE_DMA_H
#define HW_PCIE_DMA_H


// DMA parameters to tweak
static const unsigned int ACL_PCIE_DMA_MAX_PINNED_MEM_SIZE = 1024*1024;
static const unsigned int ACL_PCIE_DMA_MAX_PINNED_MEM = 64; // x PINNED_MEM_SIZE above
static const unsigned int ACL_PCIE_DMA_MAX_ATT_PER_DESCRIPTOR = 128;

// Constants matched to the HW
static const unsigned int ACL_PCIE_DMA_MAX_DONE_COUNT = (1 << 16);
static const unsigned int ACL_PCIE_DMA_MAX_ATT_PAGE_SIZE = 4*1024;
static const unsigned int ACL_PCIE_DMA_MAX_ATT_SIZE = 256;
static const unsigned int ACL_PCIE_DMA_MAX_DESCRIPTORS = 128;

static const unsigned int ACL_PCIE_DMA_ATT_PAGE_ADDR_MASK = 4*1024-1; // (ACL_PCIE_DMA_MAX_ATT_PAGE_SIZE-1);

#ifdef LINUX
#  define cl_ulong unsigned long
#endif

struct DMA_DESCRIPTOR {
   unsigned int read_address;
   unsigned int write_address;
   unsigned int bytes;
   unsigned int burst;
   unsigned int stride;
   unsigned int read_address_hi;
   unsigned int write_address_hi;
   unsigned int control;
};

struct DESCRIPTOR_UPDATE_DATA {
   unsigned int bytes;
   unsigned int att_entries;
   cl_ulong start;
};

#endif // HW_PCIE_DMA_H
