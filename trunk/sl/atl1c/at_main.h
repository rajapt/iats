/* at_main.h -- ATL1c adapter definitions
 *
 * Copyright (c) 2010 maolj <maolj@hotmail.com>.
 * All rights reserved.
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
 * Driver for Atheros(R) AR8131/AR8132/AR8152 PCI-E Ethernet Network Driver.
 *
 * This driver is heavily based on ATL1c Linux driver by Jie Yang <jie.yang@atheros.com>.
 */

#ifndef __AT_MAIN_H__
#define __AT_MAIN_H__
int atl1c_setup_ring_resources(atl1c_adapter *adapter);
void atl1c_free_ring_resources(atl1c_adapter *adapter);
void atl1c_clear_phy_int(atl1c_adapter *adapter);

void atl1c_clean_rrd(atl1c_rrd_ring *rrd_ring, atl1c_recv_ret_status *rrs, u16 num);
void atl1c_clean_rfd(atl1c_rfd_ring *rfd_ring, atl1c_recv_ret_status *rrs, u16 num);
int atl1c_alloc_rx_buffer(atl1c_adapter *adapter, const int ringid);
void atl1c_init_ring_ptrs(atl1c_adapter *adapter);
int atl1c_configure(atl1c_adapter *adapter);
void atl1c_irq_enable(atl1c_adapter *adapter);
void atl1c_irq_disable(atl1c_adapter *adapter);
int atl1c_reset_mac(atl1c_hw *hw);

s32 atl1c_alloc_tx_buffers(atl1c_adapter *adapter, atl1c_trans_queue type);
void atl1c_clean_tx_ring(atl1c_adapter *adapter,atl1c_trans_queue type);
void atl1c_clean_rx_ring(atl1c_adapter *adapter);
void atl1c_setup_mac_ctrl(atl1c_adapter *adapter);
int  atl1c_sw_init(atl1c_adapter *adapter);
void atl1c_reset_pcie(IOPCIDevice *pdev,atl1c_hw *hw, u32 flag);

int atl1c_stop_mac(atl1c_hw *hw);
void atl1c_set_aspm(atl1c_hw *hw, bool linkup);
void atl1c_enable_tx_ctrl(atl1c_hw *hw);
void atl1c_enable_rx_ctrl(atl1c_hw *hw);

atl1c_tpd_desc *atl1c_get_tpd(atl1c_adapter *adapter,atl1c_trans_queue type);
atl1c_buffer *atl1c_get_tx_buffer(atl1c_adapter *adapter, atl1c_tpd_desc *tpd);
u16 atl1c_tpd_avail(atl1c_adapter *adapter, atl1c_trans_queue type);

#endif//__AT_MAIN_H__
