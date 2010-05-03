/* at_main.h -- ATL1 adapter definitions
 *
 * Copyright(c) 2007 Atheros Corporation. All rights reserved.
 * Copyright(c) 2010 maolj <maolj@hotmail.com> All rights reserved.
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
 *
 * Driver for the Atheros(R) L1 Gigabit Ethernet Adapter.
 *
 * This driver is heavily based on ATL1 Linux driver by xiong huang <xiong.huang@atheros.com>.
 */

#ifndef _AT_MAIN_H__
#define _AT_MAIN_H__

s32 at_setup_ring_resources(at_adapter *adapter);
void at_free_ring_resources(at_adapter *adapter);
void at_setup_pcicmd(IOPCIDevice *pdev);
void at_irq_enable(at_adapter *adapter);
void at_irq_disable(at_adapter *adapter);

int at_sw_init(at_adapter *adapter);
void init_ring_ptrs(at_adapter *adapter);
s32 at_configure(at_adapter *adapter);

void at_setup_mac_ctrl(at_adapter* adapter);
void at_clean_tx_ring(at_adapter *adapter);
void at_clean_rx_ring(at_adapter *adapter);
void at_clear_phy_int(at_adapter* adapter);
u16 at_alloc_tx_buffers(at_adapter *adapter);
u16 at_alloc_rx_buffers(at_adapter *adapter);
u16 tpd_avail(struct at_tpd_ring *tpd_ring);

void at_inc_smb(at_adapter * adapter);
void at_update_mailbox(struct at_adapter* adapter);
#endif//_AT_MAIN_H__
