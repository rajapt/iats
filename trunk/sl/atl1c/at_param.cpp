/*
 * Copyright(c) 2007 Atheros Corporation. All rights reserved.
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
 */

#include "at_osdep.h"
#include "at_hw.h" 
#include "at.h" 
#include "at_param.h"

/* This is the only thing that needs to be changed to adjust the
 * maximum number of ports that the driver can manage.
 */

/* Transmit Memory count
 *
 * Valid Range: 64-2048
 *
 * Default Value: 128
 */
#define ATL1C_MIN_TX_DESC_CNT		32
#define ATL1C_MAX_TX_DESC_CNT		1024
#define ATL1C_DEFAULT_TX_DESC_CNT	1024	

/* Receive Memory Block Count
 *
 * Valid Range: 16-512
 *
 * Default Value: 128
 */
#define ATL1C_MIN_RX_MEM_SIZE		128 
#define ATL1C_MAX_RX_MEM_SIZE		1024
#define ATL1C_DEFAULT_RX_MEM_SIZE	512	


/* Interrupt Moderate Timer in units of 2 us
 *
 * Valid Range: 10-65535
 *
 * Default Value: 45000(90ms)
 */


#define AUTONEG_ADV_DEFAULT  0x2F
#define AUTONEG_ADV_MASK     0x2F
#define FLOW_CONTROL_DEFAULT FLOW_CONTROL_FULL

#define FLASH_VENDOR_DEFAULT    0
#define FLASH_VENDOR_MIN        0
#define FLASH_VENDOR_MAX        2


/*
 * atl1c_check_options - Range Checking for Command Line Parameters
 * @adapter: board private structure
 *
 * This routine checks all command line parameters for valid user
 * input.  If an invalid value is given, or if no user specified
 * value exists, a default value is used.  The final value is stored
 * in a variable in the adapter structure.
 */
void atl1c_check_options(atl1c_adapter *adapter)
{
    int  def;

    { /* Transmit Ring Size */
        def  = ATL1C_DEFAULT_TX_DESC_CNT;
        adapter->tpd_ring[0].count = (u16)def;
    }

    { /* Receive Memory Block Count */
        def  = ATL1C_DEFAULT_RX_MEM_SIZE;
        adapter->rfd_ring[0].count = (u32)def;
    }
    
    { /* Interrupt Moderate Timer  */
         def  = INT_MOD_DEFAULT_CNT * 20;
         adapter->hw.tx_imt = (u16)def;          
    }
    { /* Interrupt Moderate Timer  */
         def  = INT_MOD_DEFAULT_CNT * 5;
         adapter->hw.rx_imt = (u16)def;          
    }
    { /* MediaType */
	    def  = MEDIA_TYPE_AUTO_SENSOR;
	    adapter->hw.media_type = (u16)def;    
    }
}

