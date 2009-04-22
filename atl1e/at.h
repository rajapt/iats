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

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer */
struct at_buffer {
	//struct sk_buff *skb;
	IOBufferMemoryDescriptor *memDesc;
	u16 length;
	dma_addr_t dma;
};


struct at_page {
    // HW DMA-Address
    dma_addr_t				dma;
    u8*						addr;

    // HW Offset
    dma_addr_t			    WptrPhyAddr;
    u32*					pWptr;

    // SW Offset
    u32		                Rptr;
};


struct at_ring_header {
    /* pointer to the descriptor ring memory */
    void *desc;
    /* physical adress of the descriptor ring */
    dma_addr_t dma;
    /* length of descriptor ring in bytes */
    unsigned int size;
};

/* board specific private data structure */
          
struct at_adapter {
    /* OS defined structs */
    IOPCIDevice	*pdev;

    u32 wol;
    u16 link_speed;
    u16 link_duplex;
    spinlock_t stats_lock;
    spinlock_t tx_lock;
    atomic_t irq_sem;//
    unsigned long cfg_phy;

    // All Descriptor memory
    IOBufferMemoryDescriptor *memDesc;
    dma_addr_t  	ring_dma;
    void*           ring_vir_addr;
    int             ring_size;
    
    TpdDescr*		tpd_ring;
    dma_addr_t		tpd_ring_dma;
	u16 			tpd_ring_size;	// number of TPDs within the TPD ring
	u16 			tpd_next_use;
	atomic_t		tpd_next_clean;
	at_buffer		*tx_buffer_info;
	dma_addr_t		tpd_cmb_dma;
	u32*			tpd_cmb;
	
	
	at_page			rxf_page[4][2];
	u8				rxf_using[4];
	u16				rxf_nxseq[4];	// next sequence number
	u32				rxf_length;		// bytes length of rxf page
	
	int	num_rx_queues;
    
    /* Interrupt Moderator timer ( 2us resolution) */
    u16 imt;
    /* Interrupt Clear timer (2us resolution) */
    u16 ict;
    
	u32 hw_csum_err;
    
	unsigned long flags;
    /* structs defined in at_hw.h */
    u32 bd_number;     // board number;
    bool pci_using_64;
    bool have_msi;
    at_hw hw;

    u32 usr_cmd;
//    u32 regs_buff[AT_REGS_LEN];
    u32 pci_state[16];

    u32* config_space;
};

enum at_state_t {
	__AT_TESTING,
	__AT_RESETTING,
	__AT_DOWN
};


#endif//_ATHEROS_H__

