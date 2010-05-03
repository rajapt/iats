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
 *
 * There are a lot of defines in here that are unused and/or have cryptic
 * names.  Please leave them alone, as they're the closest thing we have
 * to a spec from Atheros at present. *ahem* -- CHS
 */

#include "at_osdep.h"
#include "at_hw.h" 
#include "at.h" 
#include "at_param.h"

/* This is the only thing that needs to be changed to adjust the
 * maximum number of ports that the driver can manage.
 */



#define AUTONEG_ADV_DEFAULT  0x2F
#define AUTONEG_ADV_MASK     0x2F
#define FLOW_CONTROL_DEFAULT FLOW_CONTROL_FULL

/* Interrupt Moderate Timer in units of 2 us
 *
 * Valid Range: 10-65535
 *
 * Default Value: 45000(90ms)
 */
#define DEFAULT_INT_MOD_CNT     100 // 200us
#define MAX_INT_MOD_CNT         65000
#define MIN_INT_MOD_CNT         50

/* FlashVendor
 * Valid Range: 0-2
 * 0 - Atmel
 * 1 - SST
 * 2 - ST
 */
#define FLASH_VENDOR_DEFAULT    0
#define FLASH_VENDOR_MIN        0
#define FLASH_VENDOR_MAX        2

/* Receive Descriptor Count
 *
 * Valid Range: 64-4096
 *
 * Default Value: 256
 */
#define AT_DEFAULT_RFD      512
#define AT_MIN_RFD          128
#define AT_MAX_RFD          2047

/* Transmit Descriptor Count
 *
 * Valid Range: 64-2048 
 *
 * Default Value: 128
 */
#define AT_DEFAULT_TPD      256
#define AT_MAX_TPD          1023
#define AT_MIN_TPD          64

/* User Specified MediaType Override
 *
 * Valid Range: 0-4
 *  - 0    - auto-negotiate at all supported speeds
 *  - 1    - only link at 100Mbps Full Duplex
 *  - 2    - only link at 100Mbps Half Duplex
 *  - 3    - only link at 10Mbps Full Duplex
 *  - 4    - only link at 10Mbps Half Duplex
 * Default Value: 0
 */


/**
 * at_check_options - Range Checking for Command Line Parameters
 * @adapter: board private structure
 *
 * This routine checks all command line parameters for valid user
 * input.  If an invalid value is given, or if no user specified
 * value exists, a default value is used.  The final value is stored
 * in a variable in the adapter structure.
 **/

void
at_check_options(struct at_adapter *adapter)
{
    int  def;
	
    { /* Transmit Descriptor Count */
        def  = AT_DEFAULT_TPD;
        struct at_tpd_ring *tpd_ring = &adapter->tpd_ring;
        tpd_ring->count = (uint16_t)def;
    }
	
    { /* Receive Descriptor Count */
        def  = AT_DEFAULT_RFD;
        struct at_rfd_ring *rfd_ring = &adapter->rfd_ring;
        struct at_rrd_ring *rrd_ring = &adapter->rrd_ring;
		
        rfd_ring->count = rrd_ring->count = (uint16_t)def;
    }
    
    { /* Interrupt Moderate Timer  */
        def  = DEFAULT_INT_MOD_CNT;
        adapter->imt = (uint16_t)def;          
    }
    { /* Flsh Vendor */
        def  = FLASH_VENDOR_DEFAULT;
        adapter->hw.flash_vendor = (uint8_t)def;
    }
    { /* MediaType */
        def  = MEDIA_TYPE_AUTO_SENSOR;
        adapter->hw.MediaType = (uint16_t)def;    
    }
	
}

