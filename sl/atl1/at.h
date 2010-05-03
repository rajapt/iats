/*
 * Copyright(c) 2007 Atheros Corporation. All rights reserved.
 * Copyright(c) 2007 xiong huang <xiong.huang@atheros.com>
 *
 * Derived from Intel e1000 driver
 * Copyright(c) 1999 - 2005 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * There are a lot of defines in here that are unused and/or have cryptic
 * names.  Please leave them alone, as they're the closest thing we have
 * to a spec from Atheros at present. *ahem* -- CHS
 */

#ifndef _ATHEROS_H__
#define _ATHEROS_H__


#define BAR_0   0
#define BAR_1   1
#define BAR_5   5

#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/network/IOEthernetController.h>

#include "at_hw.h"

#define AT_DESC_UNUSED(R) \
((((R)->next_to_clean > (R)->next_to_use) ? 0 : (R)->count) + \
(R)->next_to_clean - (R)->next_to_use - 1)

#define AT_DESC_USED(R) \
(((R)->next_to_clean > (R)->next_to_use) ?  \
((R)->count+(R)->next_to_use-(R)->next_to_clean+1) : \
((R)->next_to_use-(R)->next_to_clean+1))


#define AT_GET_DESC(R, i, type)     (&(((type *)((R)->desc))[i]))
#define AT_RFD_DESC(R, i)       AT_GET_DESC(R, i, rx_free_desc_t)
#define AT_TPD_DESC(R, i)       AT_GET_DESC(R, i, TpdDescr)
#define AT_RRD_DESC(R, i)       AT_GET_DESC(R, i, rx_return_desc_t)

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer */
struct at_buffer {
    //struct sk_buff *skb;
    IOBufferMemoryDescriptor *memDesc;
    uint16_t length;
    uint16_t alloced;
    dma_addr_t dma;
};

struct at_tpd_ring {
    void *desc;         /* pointer to the descriptor ring memory */
    dma_addr_t dma;     /* physical adress of the descriptor ring */
    uint16_t size;      /* length of descriptor ring in bytes */
    uint16_t count;     /* number of descriptors in the ring */
    
    uint16_t hw_idx;    /* hardware index */
    atomic_t next_to_clean;
    atomic_t next_to_use;
    struct at_buffer *buffer_info;
    uint32_t buffer_ring_size;
};

struct at_rfd_ring {
    void *desc;
    dma_addr_t dma;
    uint16_t size;
    uint16_t count;
    atomic_t next_to_use;
    uint16_t next_to_clean;
    struct at_buffer *buffer_info;
};

struct at_rrd_ring {
    void *desc;
    dma_addr_t dma;
    unsigned int size;
    uint16_t count;
    uint16_t next_to_use; 
    atomic_t next_to_clean;
};

struct at_ring_header {
    IOBufferMemoryDescriptor *memDesc;
    /* pointer to the descriptor ring memory */
    void *desc;
    /* physical adress of the descriptor ring */
    dma_addr_t dma;
    /* length of descriptor ring in bytes */
    unsigned int size;
};

struct at_cmb {
    coals_msg_block_t* cmb;
    dma_addr_t dma;
};
struct at_smb {
    stats_msg_block_t* smb;
    dma_addr_t dma;
};

/* Statistics counters */
struct at_sft_stats {
    uint64_t rx_packets;
    uint64_t tx_packets;
    uint64_t rx_bytes;
    uint64_t tx_bytes;
    uint64_t multicast;
    uint64_t collisions;
    uint64_t rx_errors;
    uint64_t rx_length_errors;
    uint64_t rx_crc_errors;
    uint64_t rx_frame_errors;
    uint64_t rx_fifo_errors;
    uint64_t rx_missed_errors;
    uint64_t tx_errors;
    uint64_t tx_fifo_errors;
    uint64_t tx_aborted_errors;
    uint64_t tx_window_errors; 
    uint64_t tx_carrier_errors;
    
    uint64_t tx_pause;      // The number of Pause packet transmitted.
    uint64_t excecol;       // The number of transmit packets aborted due to excessive collisions.
    uint64_t deffer;        // The number of packets transmitted that is deferred.
    uint64_t scc;           // The number of packets subsequently transmitted successfully with a single prior collision.
    uint64_t mcc;           // The number of packets subsequently transmitted successfully with multiple prior collisions.
    uint64_t latecol;       // The number of packets transmitted with late collisions.
    uint64_t tx_underun;    // The number of transmit packets aborted due to transmit FIFO underrun, or TRD FIFO underrun
    uint64_t tx_trunc;      // The number of transmit packets truncated due to size exceeding MTU, regardless if it is truncated by Selene or not. 
	// (The name is not really reflects the meaning in this case here.)
    
    uint64_t rx_pause;      // The number of Pause packet received.
    uint64_t rx_rrd_ov;
    uint64_t rx_trunc;
};


/* board specific private data structure */

struct at_adapter {
    /* OS defined structs */
    IOPCIDevice	*pdev;
    IONetworkStats *net_stats;
    struct at_sft_stats soft_stats;
    
    uint32_t rx_buffer_len;
    uint32_t wol;
    uint16_t link_speed;
    uint16_t link_duplex;
    //spinlock_t stats_lock;
    //spinlock_t tx_lock;
    atomic_t irq_sem;//
    //unsigned long cfg_phy;
	
    // All descriptor rings' memory
    struct at_ring_header ring_header;
    
    /* TX */
    
    struct at_tpd_ring tpd_ring;
    //spinlock_t mb_lock;
    
	
    /* RX */
    struct at_rfd_ring rfd_ring;
    struct at_rrd_ring rrd_ring;
    uint64_t hw_csum_err;
    uint64_t hw_csum_good;
	
    /* Interrupt Moderator timer ( 2us resolution) */
    uint16_t imt;
    /* Interrupt Clear timer (2us resolution) */
    uint16_t ict;
    
	
    /* structs defined in at_hw.h */
    uint32_t bd_number;     // board number;
    boolean_t pci_using_64;
    boolean_t have_msi;
    struct at_hw hw;
    struct at_smb smb;
    struct at_cmb cmb;
	
    //unsigned long flags;
    int num_rx_queues;
	
    u32 pci_state[16];
	
    u32* config_space;
	
};

enum at_state_t {
    __AT_TESTING,
    __AT_RESETTING,
    __AT_DOWN
};


#endif//_ATHEROS_H__

