/*
 * Copyright(c) 2007 Atheros Corporation. All rights reserved.
 * Copyright(c) 2006 xiong huang <xiong.huang@atheros.com>
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
    IOPCIDevice  *pdev;
    
    u32 wol;
    u16 link_speed;
    u16 link_duplex;
    spinlock_t stats_lock;
    spinlock_t tx_lock;
    atomic_t irq_sem;//

    unsigned long cfg_phy;

    bool mac_disabled;


    // All Descriptor memory
    dma_addr_t  ring_dma;
    void*               ring_vir_addr;
    int                 ring_size;
    IOBufferMemoryDescriptor *memDesc;
	    
    tx_pkt_header_t* txd_ring;
    dma_addr_t           txd_dma;
    
    tx_pkt_status_t* txs_ring;
    dma_addr_t           txs_dma;
    
    rx_desc_t*           rxd_ring;
    dma_addr_t       rxd_dma;
    
    u32 txd_ring_size;         // bytes per unit
    u32 txs_ring_size;         // dwords per unit
    u32 rxd_ring_size;         // 1536bytes per unit

    // read /write ptr:
    // host
    u32 txd_write_ptr;
    u32 txs_next_clear;
    u32 rxd_read_ptr;
    
    // nic
    atomic_t txd_read_ptr;
    atomic_t txs_write_ptr;
    u32 rxd_write_ptr;
    
    
    /* Interrupt Moderator timer ( 2us resolution) */
    u16 imt;
    /* Interrupt Clear timer (2us resolution) */
    u16 ict;
    
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

