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
#include <IOKit/pci/IOPCIDevice.h>
#include "at_hw.h"
#include "at.h"

#define LBYTESWAP( a )  ( ( ( (a) & 0x00ff00ff ) << 8 ) | ( ( (a) & 0xff00ff00 ) >> 8 ) )
#define LONGSWAP( a )   ( ( LBYTESWAP( a ) << 16 ) | ( LBYTESWAP( a ) >> 16 ) )
#define SHORTSWAP( a )  ( ( (a) << 8 ) | ( (a) >> 8 ) )


void at_init_pcie(at_hw *hw);

/* 
 * The little-endian AUTODIN II ethernet CRC calculations.
 * A big-endian version is also available.
 * This is slow but compact code.  Do not use this routine 
 * for bulk data, use a table-based routine instead.
 * This is common code and should be moved to net/core/crc.c.
 * Chips may use the upper or lower CRC bits, and may reverse 
 * and/or invert them.  Select the endian-ness that results 
 * in minimal calculations.
 */
u32
ether_crc_le(int length, unsigned char *data)
{
    u32 crc = ~0;  /* Initial value. */
    while(--length >= 0) {
        unsigned char current_octet = *data++;
        int bit;
        for (bit = 8; --bit >= 0; current_octet >>= 1) {
            if ((crc ^ current_octet) & 1) {
                crc >>= 1;
                crc ^= 0xedb88320;
            } 
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

/********************************************************************
* Reset the transmit and receive units; mask and clear all interrupts.
* 
* hw - Struct containing variables accessed by shared code
* return : AT_SUCCESS  or  idle status (if error)
********************************************************************/
s32
at_reset_hw(at_adapter* adapter)
{
    u32 icr;
    u16 pci_cfg_cmd_word;
    int i;
	IOPCIDevice *pdev = adapter->pdev;
    at_hw *hw = &adapter->hw;
	
    DEBUGFUNC("at_reset_hw()\n");

    /* Workaround for PCI problem when BIOS sets MMRBC incorrectly. */
	pci_cfg_cmd_word =pdev->configRead16(PCI_REG_COMMAND);
    if ((pci_cfg_cmd_word&
           (CMD_IO_SPACE|CMD_MEMORY_SPACE|CMD_BUS_MASTER))
        != (CMD_IO_SPACE|CMD_MEMORY_SPACE|CMD_BUS_MASTER)) {
        pci_cfg_cmd_word |= 
           (CMD_IO_SPACE|CMD_MEMORY_SPACE|CMD_BUS_MASTER);
        pdev->configWrite16( PCI_REG_COMMAND, pci_cfg_cmd_word);
    }


    /* Clear Interrupt mask to stop board from generating
     * interrupts & Clear any pending interrupt events 
     */
//    AT_WRITE_REG(hw, REG_IMR, 0);
//    AT_WRITE_REG(hw, REG_ISR, 0xffffffff);

    /* Issue Soft Reset to the MAC.  This will reset the chip's
     * transmit, receive, DMA.  It will not effect
     * the current PCI configuration.  The global reset bit is self-
     * clearing, and should clear within a microsecond.
     */
    AT_WRITE_REG(hw, REG_MASTER_CTRL, MASTER_CTRL_LED_MODE|MASTER_CTRL_SOFT_RST);
    //wmb();

   
    msec_delay(1); // delay about 1ms 

    /* Wait at least 10ms for All module to be Idle 
     */
    for (i=0; i < 10; i++)
    {
        icr = AT_READ_REG(hw, REG_IDLE_STATUS);
        if (!icr)
            break;
        msec_delay(1); // delay 1 ms
        //cpu_relax();
    }

    if (icr) 
    {
        DEBUGOUT("MAC state machine cann't be idle since disabled for 10ms second\n");
        return icr;
    }

    return AT_SUCCESS;
}


/*********************************************************************
* Reads the adapter's MAC address from the EEPROM 
*
* hw - Struct containing variables accessed by shared code
*********************************************************************/
s32
at_read_mac_addr(at_hw * hw)
{
    u16  i;
    
    DEBUGFUNC("at_read_mac_addr()\n");
    
    if (get_permanent_address(hw)) {
        // for test
        hw->perm_mac_addr[0] = 0x00;
        hw->perm_mac_addr[1] = 0x13;
        hw->perm_mac_addr[2] = 0x74;
        hw->perm_mac_addr[3] = 0x00;
        hw->perm_mac_addr[4] = 0x5c;
        hw->perm_mac_addr[5] = 0x38;
    } 
    
    for(i = 0; i < NODE_ADDRESS_SIZE; i++)
        hw->mac_addr[i] = hw->perm_mac_addr[i];
    return AT_SUCCESS;
}

/*********************************************************************
* Hashes an address to determine its location in the multicast table
*
* hw - Struct containing variables accessed by shared code
* mc_addr - the multicast address to hash
*********************************************************************/
/* 
 * at_hash_mc_addr
 *  purpose
 *      set hash value for a multicast address
 *      hash calcu processing :
 *          1. calcu 32bit CRC for multicast address
 *          2. reverse crc with MSB to LSB
 */
u32
at_hash_mc_addr(
        at_hw *hw,
        u8 *mc_addr)
{
    u32 crc32, value=0;
    int i;

    crc32 = ether_crc_le(6, mc_addr);
    crc32 = ~crc32;
    for (i=0; i<32; i++)
    {
        value |= (((crc32>>i)&1)<<(31-i));
    }

    return value;
}


/********************************************************************
* Sets the bit in the multicast table corresponding to the hash value.
*
* hw - Struct containing variables accessed by shared code
* hash_value - Multicast address hash value
********************************************************************/
void
at_hash_set(
            at_hw *hw, 
            u32 hash_value)
{
    u32 hash_bit, hash_reg;
    u32 mta;
    
    /* The HASH Table  is a register array of 2 32-bit registers.
    * It is treated like an array of 64 bits.  We want to set
    * bit BitArray[hash_value]. So we figure out what register
    * the bit is in, read it, OR in the new bit, then write
    * back the new value.  The register is determined by the
    * upper 7 bits of the hash value and the bit within that
    * register are determined by the lower 5 bits of the value.
    */
    hash_reg = (hash_value >> 31) & 0x1;
    hash_bit = (hash_value >> 26) & 0x1F;
    
    mta = AT_READ_REG_ARRAY(hw, REG_RX_HASH_TABLE, hash_reg);
    
    mta |= (1 << hash_bit);
    
    AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, hash_reg, mta);
}


/*
 * at_init_pcie - init PCIE module
 */
void at_init_pcie(at_hw *hw)
{
	u32 value;
	/* comment 2lines below to save more power when sususpend 
	value = LTSSM_TEST_MODE_DEF;
    AT_WRITE_REG(hw, REG_LTSSM_TEST_MODE, value);
    */
    
	/* pcie flow control mode change */
	value = AT_READ_REG(hw, 0x1008);
	value |= 0x8000;
	AT_WRITE_REG(hw, 0x1008, value);
}

/********************************************************************
* Performs basic configuration of the adapter.
*
* hw - Struct containing variables accessed by shared code
* Assumes that the controller has previously been reset and is in a
* post-reset uninitialized state. Initializes multicast table, 
* and  Calls routines to setup link
* Leaves the transmit and receive units disabled and uninitialized.
********************************************************************/
s32
at_init_hw(at_hw *hw)
{
    s32 ret_val = 0;
    
    DEBUGFUNC("at_init_hw()\n");
    
    at_init_pcie(hw);
    	
    	
    /* Zero out the Multicast HASH table */
//    DEBUGOUT("Zeroing the MTA\n");
    /* clear the old settings from the multicast hash table */
    AT_WRITE_REG(hw, REG_RX_HASH_TABLE, 0);
    AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 1, 0);
    
    ret_val = at_phy_init(hw);
    
    DEBUGOUT1("at_init_hw: ret: %d\n", ret_val);

    return ret_val;
}   

s32 
at_restart_autoneg(at_hw *hw)
{
    s32 ret_val;
    ret_val = at_write_phy_reg(hw, MII_ADVERTISE, 
                               hw->mii_autoneg_adv_reg);
    if(ret_val)
        return ret_val;

   if (hw->nic_type == athr_l1e ||
      hw->nic_type == athr_l2e_revA ) {                                    
	ret_val = at_write_phy_reg(hw, 
				MII_AT001_CR, 
				hw->mii_1000t_ctrl_reg);
    }
	
    ret_val = at_write_phy_reg(hw, MII_BMCR, 
                    MII_CR_RESET|MII_CR_AUTO_NEG_EN|MII_CR_RESTART_AUTO_NEG);
    return ret_val;
}
    
    
/******************************************************************************
* Detects the current speed and duplex settings of the hardware.
*
* hw - Struct containing variables accessed by shared code
* speed - Speed of the connection
* duplex - Duplex setting of the connection
*****************************************************************************/
s32
at_get_speed_and_duplex(
        at_hw *hw,
        u16 *speed,
        u16 *duplex)
{
    s32 ret_val;
    u16 phy_data;

    DEBUGFUNC("at_get_speed_and_duplex()\n");

    // ; --- Read   PHY Specific Status Register (17)
    ret_val = at_read_phy_reg(hw, MII_AT001_PSSR, &phy_data);
    if (ret_val)
        return ret_val;
    
    if (!(phy_data & MII_AT001_PSSR_SPD_DPLX_RESOLVED))
        return AT_ERR_PHY_RES;
    
    switch(phy_data&MII_AT001_PSSR_SPEED) {
    case MII_AT001_PSSR_1000MBS:
        *speed = SPEED_1000;
        DEBUGOUT("1000 Mbps, ");
        break;
    case MII_AT001_PSSR_100MBS:
        *speed = SPEED_100;
        DEBUGOUT("100 Mbs, ");
        break;
    case MII_AT001_PSSR_10MBS:
        *speed = SPEED_10;
        DEBUGOUT("10 Mbs, ");
        break;
    default:
        DEBUGOUT("Error Speed !\n");
        return AT_ERR_PHY_SPEED;
        break;
    }
    
    if (phy_data & MII_AT001_PSSR_DPLX) {
        *duplex = FULL_DUPLEX;
        DEBUGOUT(" Full Duplex\n");
    } else {
        *duplex = HALF_DUPLEX;
        DEBUGOUT(" Half Duplex\n");
    }

    return AT_SUCCESS;
}

/*********************************************************************
* Reads the value from a PHY register
* hw - Struct containing variables accessed by shared code
* reg_addr - address of the PHY register to read
*********************************************************************/
s32
at_read_phy_reg(
            at_hw *hw,
            u16 reg_addr,
            u16 *phy_data)
{
    u32 val;
    int i;
    
    DEBUGFUNC("at_read_phy_reg()\n");

    val = ((u32)(reg_addr&MDIO_REG_ADDR_MASK)) 
                << MDIO_REG_ADDR_SHIFT |
            MDIO_START |
            MDIO_SUP_PREAMBLE |
            MDIO_RW |
            MDIO_CLK_25_4 << MDIO_CLK_SEL_SHIFT;
    AT_WRITE_REG(hw, REG_MDIO_CTRL, val);

    //wmb();
    
    for (i=0; i<MDIO_WAIT_TIMES; i++) {
        usec_delay(2);
        val = AT_READ_REG(hw, REG_MDIO_CTRL);
        if (!(val&(MDIO_START|MDIO_BUSY))) {
            break;
        }
        //wmb();
    }
    if (!(val&(MDIO_START|MDIO_BUSY))) {
        *phy_data = (u16)val;
        return AT_SUCCESS; 
    }   

    return AT_ERR_PHY;
}

/********************************************************************
* Writes a value to a PHY register
* hw - Struct containing variables accessed by shared code
* reg_addr - address of the PHY register to write
* data - data to write to the PHY
********************************************************************/
s32
at_write_phy_reg(
        at_hw *hw,
        u32 reg_addr,
        u16 phy_data)
{
    int i;
    u32 val;
    
    val =   ((u32)(phy_data & MDIO_DATA_MASK)) << MDIO_DATA_SHIFT |
            (reg_addr&MDIO_REG_ADDR_MASK) << MDIO_REG_ADDR_SHIFT |
            MDIO_SUP_PREAMBLE |
            MDIO_START |
            MDIO_CLK_25_4 << MDIO_CLK_SEL_SHIFT;
    AT_WRITE_REG(hw, REG_MDIO_CTRL, val);
//    DEBUGOUT1("phy write 0x%x <- 0x%x, value = 0x%x\n", i
//              reg_addr, 
//              phy_data, 
//              val);
    
    //wmb();
    
    for (i=0; i<MDIO_WAIT_TIMES; i++) {
        usec_delay(2);
        val = AT_READ_REG(hw, REG_MDIO_CTRL);
        if (!(val&(MDIO_START|MDIO_BUSY))) {
            break;
        }
        //wmb();
    }

    if (!(val&(MDIO_START|MDIO_BUSY)))
        return AT_SUCCESS;

    return AT_ERR_PHY;
}

/********************************************************************
* Configures PHY autoneg and flow control advertisement settings
*
* hw - Struct containing variables accessed by shared code
********************************************************************/
s32
at_phy_setup_autoneg_adv(at_hw *hw)
{
    s32 ret_val;
    u16 mii_autoneg_adv_reg;
    u16 mii_1000t_ctrl_reg;

    DEBUGFUNC("at_phy_setup_autoneg_adv()\n");

    if (0 != hw->mii_autoneg_adv_reg)
        return AT_SUCCESS;
    
    /* Read the MII Auto-Neg Advertisement Register (Address 4/9). */
    mii_autoneg_adv_reg = MII_AR_DEFAULT_CAP_MASK;
	mii_1000t_ctrl_reg = MII_AT001_CR_1000T_DEFAULT_CAP_MASK;
    
    /* Need to parse autoneg_advertised  and set up
    * the appropriate PHY registers.  First we will parse for
    * autoneg_advertised software override.  Since we can advertise
    * a plethora of combinations, we need to check each bit
    * individually.
    */
    
    /* First we clear all the 10/100 mb speed bits in the Auto-Neg
    * Advertisement Register (Address 4) and the 1000 mb speed bits in
    * the  1000Base-T Control Register (Address 9).
    */
    mii_autoneg_adv_reg &= ~MII_AR_SPEED_MASK;
    mii_1000t_ctrl_reg &= ~MII_AT001_CR_1000T_SPEED_MASK;
    
    /* Need to parse MediaType and setup the 
     * appropriate PHY registers.
     */
    switch (hw->MediaType)
    {
    case MEDIA_TYPE_AUTO_SENSOR:
        mii_autoneg_adv_reg |= (MII_AR_10T_HD_CAPS|
                                MII_AR_10T_FD_CAPS|
                                MII_AR_100TX_HD_CAPS|
                                MII_AR_100TX_FD_CAPS);
	if (hw->nic_type == athr_l1e) {
	    mii_1000t_ctrl_reg |= MII_AT001_CR_1000T_FD_CAPS; 
	}
        hw->autoneg_advertised = ADVERTISE_10_HALF|
        			ADVERTISE_10_FULL |
        			ADVERTISE_100_HALF|
        			ADVERTISE_100_FULL;
        if (hw->nic_type == athr_l1e) {
            hw->autoneg_advertised |= ADVERTISE_1000_FULL;
        }
        break;
    case MEDIA_TYPE_1000M_FULL:
        if (hw->nic_type != athr_l1e)
            break; 
        mii_1000t_ctrl_reg |= MII_AT001_CR_1000T_FD_CAPS;
        hw->autoneg_advertised |= ADVERTISE_1000_FULL;
        break;
 
    case MEDIA_TYPE_100M_FULL:
        mii_autoneg_adv_reg |= MII_AR_100TX_FD_CAPS;
        hw->autoneg_advertised = ADVERTISE_100_FULL;
        break;
        
    case MEDIA_TYPE_100M_HALF:
        mii_autoneg_adv_reg |= MII_AR_100TX_HD_CAPS;
        hw->autoneg_advertised = ADVERTISE_100_HALF;
        break;
        
    case MEDIA_TYPE_10M_FULL:
        mii_autoneg_adv_reg |= MII_AR_10T_FD_CAPS;
        hw->autoneg_advertised = ADVERTISE_10_FULL;
        break;
        
    default:
        mii_autoneg_adv_reg |= MII_AR_10T_HD_CAPS;
        hw->autoneg_advertised = ADVERTISE_10_HALF;
        break;
    }
     							
    /* flow control fixed to enable all */
    mii_autoneg_adv_reg |= (MII_AR_ASM_DIR | MII_AR_PAUSE);

    hw->mii_autoneg_adv_reg = mii_autoneg_adv_reg;
	hw->mii_1000t_ctrl_reg = mii_1000t_ctrl_reg;
    
    ret_val = at_write_phy_reg(hw, 
                               MII_ADVERTISE, 
                               mii_autoneg_adv_reg);
    if(ret_val)
        return ret_val;

   if (hw->nic_type == athr_l1e ||
       hw->nic_type == athr_l2e_revA ) {                                    
           ret_val = at_write_phy_reg(hw, 
	        	MII_AT001_CR, 
			mii_1000t_ctrl_reg);
    }
	
    if (ret_val)
	return ret_val;
    
    return AT_SUCCESS;
}

s32
at_phy_init(at_hw* hw)
{
    s32 ret_val;
    u16 phy_val;
	
    if (hw->phy_configured) {
        if (hw->re_autoneg) {
            hw->re_autoneg = false;
            return at_restart_autoneg(hw);
        }
        return 0;
    }
		
    // RESET GPHY Core
    AT_WRITE_REGW(hw, REG_GPHY_CTRL, GPHY_CTRL_DEFAULT);
    msec_delay(2);
    AT_WRITE_REGW(hw, REG_GPHY_CTRL, GPHY_CTRL_DEFAULT|GPHY_CTRL_EXT_RESET);
    msec_delay(2);
    
	/* patches */
	/* p1. eable hibernation mode */
	ret_val = at_write_phy_reg(hw, MII_DBG_ADDR, 0xB);
    if (ret_val) 	return ret_val;
	ret_val = at_write_phy_reg(hw, MII_DBG_DATA, 0xBC00);
    if (ret_val) 	return ret_val;    	
	/* p2. set Class A/B for all modes */
	ret_val = at_write_phy_reg(hw, MII_DBG_ADDR, 0);
    if (ret_val) 	return ret_val;
        /* remove Class AB, just use ClassA(0x02ef) */
	//phy_val = hw->emi_ca ? 0x02ef : 0x02df;
	phy_val = 0x02ef; /* ClassA */
	ret_val = at_write_phy_reg(hw, MII_DBG_DATA, phy_val);
    if (ret_val) 	return ret_val;    		
	/* p3. 10B ??? */
	ret_val = at_write_phy_reg(hw, MII_DBG_ADDR, 0x12);
    if (ret_val) 	return ret_val;
	ret_val = at_write_phy_reg(hw, MII_DBG_DATA, 0x4C04);
    if (ret_val) 	return ret_val; 	
	/* p4. 1000T power */
	ret_val = at_write_phy_reg(hw, MII_DBG_ADDR, 0x4);
    if (ret_val) 	return ret_val;
	ret_val = at_write_phy_reg(hw, MII_DBG_DATA, 0x8BBB);
    if (ret_val) 	return ret_val;
    	
	ret_val = at_write_phy_reg(hw, MII_DBG_ADDR, 0x5);
    if (ret_val) 	return ret_val;
	ret_val = at_write_phy_reg(hw, MII_DBG_DATA, 0x2C46);
    if (ret_val) 	return ret_val;
	    		
	msec_delay(1);
	
    /*Enable PHY LinkChange Interrupt */
    ret_val = at_write_phy_reg(hw, 18, 0xC00);
    if (ret_val)
        return ret_val;

    /* setup AutoNeg parameters */
/*  //move to atSetupLink() function
   ret_val = at_phy_setup_autoneg_adv(hw);
    if(ret_val) {
        DEBUGOUT("Error Setting up Auto-Negotiation\n");
        return ret_val;
    }
*/    
    /* SW.Reset & En-Auto-Neg to restart Auto-Neg*/
/*    DEBUGOUT("Restarting Auto-Neg\n");
    ret_val = at_phy_commit(hw);
    if (ret_val) {
        DEBUGOUT("Error Resetting the phy\n");
        return ret_val;
    }
 */
    hw->phy_configured = true;
    
    return ret_val;
}

/*******************************************************************
* Resets the PHY and make all config validate
*
* hw - Struct containing variables accessed by shared code
*
* Sets bit 15 and 12 of the MII Control regiser (for F001 bug)
*******************************************************************/
s32
at_phy_commit(at_hw *hw)
{
    s32 ret_val;
    u16 phy_data;
    
    DEBUGFUNC("at_phy_commit()\n");

    phy_data = MII_CR_RESET | MII_CR_AUTO_NEG_EN | MII_CR_RESTART_AUTO_NEG;
    
    ret_val = at_write_phy_reg(hw, MII_BMCR, phy_data);
    if (ret_val) { // bug fixed
        u32 val;
        int i;
        /**************************************
        * pcie serdes link may be down !
        **************************************/
        DEBUGOUT("Auto-Neg make pcie phy link down !\n");

        for (i=0; i < 25; i++) {
            msec_delay(1);
            val = AT_READ_REG(hw, REG_MDIO_CTRL);
            if (!(val&(MDIO_START|MDIO_BUSY))) {
                break;
            }
        }
     
        if (0 != (val&(MDIO_START|MDIO_BUSY))) {
            AT_ERR("pcie linkdown at least for 25ms !\n");
            return ret_val;

        DEBUGOUT1("pcie linkup after %dms\n", i);
        }
    }
    return AT_SUCCESS;
}

void
set_mac_addr(at_hw* hw)
{
    u32 value;
    // 00-0B-6A-F6-00-DC
    // 0:  6AF600DC   1: 000B
    //  low dword
    value = (((u32)hw->mac_addr[2]) << 24) |
            (((u32)hw->mac_addr[3]) << 16) |
            (((u32)hw->mac_addr[4]) << 8 ) |
            (((u32)hw->mac_addr[5])      ) ;
    AT_WRITE_REG_ARRAY(hw, REG_MAC_STA_ADDR, 0, value);
    // hight dword
    value = (((u32)hw->mac_addr[0]) << 8 ) |
            (((u32)hw->mac_addr[1])      ) ;
    AT_WRITE_REG_ARRAY(hw, REG_MAC_STA_ADDR, 1, value);
}


/*************************************** function about EEPROM **********************************/
/*
 * check_eeprom_exist
 * return 0 if eeprom exist
 */
int 
check_eeprom_exist(at_hw* hw)
{
    u32 value;
    
    value = AT_READ_REG(hw, REG_SPI_FLASH_CTRL);
    if (value & SPI_FLASH_CTRL_EN_VPD) 
    {
        value &= ~SPI_FLASH_CTRL_EN_VPD;
        AT_WRITE_REG(hw, REG_SPI_FLASH_CTRL, value);
    }
    value = AT_READ_REGW(hw, REG_PCIE_CAP_LIST);
    return ((value & 0xFF00) == 0x6C00) ? 0 : 1;
}

/*
 * get_permanent_address
 * return 0 if get valid mac address, 
 */
int 
get_permanent_address(at_hw *hw)
{
#define AT_TWSI_EEPROM_TIMEOUT 10
    u32 Addr[2];
    u32 i;
    u32 twsi_ctrl_data;
    u8  EthAddr[NODE_ADDRESS_SIZE];
    
    if (is_valid_ether_addr(hw->perm_mac_addr))
        return 0;
    // init
    Addr[0] = Addr[1] = 0;    
    
    if (!check_eeprom_exist(hw)) { // eeprom exist
        i = 0;
        twsi_ctrl_data = AT_READ_REG(hw, REG_TWSI_CTRL);
        twsi_ctrl_data |= TWSI_CTRL_SW_LDSTART; 
        AT_WRITE_REG(hw, REG_TWSI_CTRL, twsi_ctrl_data);
        for (i = 0; i < AT_TWSI_EEPROM_TIMEOUT; i++) {
            msec_delay(5);
            twsi_ctrl_data = AT_READ_REG(hw, REG_TWSI_CTRL);
            if ((twsi_ctrl_data & TWSI_CTRL_SW_LDSTART) == 0) {
                break;
            }
        }
        if (i >= AT_TWSI_EEPROM_TIMEOUT)
            return -1;
    } 
    
    Addr[0] = AT_READ_REG(hw,REG_MAC_STA_ADDR);
    Addr[1] = AT_READ_REG(hw,REG_MAC_STA_ADDR+4);
    *(u32*) &EthAddr[2] = LONGSWAP(Addr[0]);
    *(u16*) &EthAddr[0] = SHORTSWAP(*(u16*)&Addr[1]);
    
    if (is_valid_ether_addr(EthAddr)) {
        memcpy(hw->perm_mac_addr, EthAddr, NODE_ADDRESS_SIZE);
        return 0;
    }
    
    return 1;
#undef AT_TWSI_EEPROM_TIMEOUT
}

bool 
write_eeprom(at_hw* hw, u32 offset, u32 value)
{
    return true;
}

bool
read_eeprom(at_hw* hw, u32 Offset, u32* pValue)
{
    int i;
    u32    Control;
    
    if (Offset&3)   return false; //address do not align

    AT_WRITE_REG(hw, REG_VPD_DATA, 0);
    Control = (Offset&VPD_CAP_VPD_ADDR_MASK)<<VPD_CAP_VPD_ADDR_SHIFT;
    AT_WRITE_REG(hw, REG_VPD_CAP, Control);
    
    for (i=0; i < 10; i++)
    {
        msec_delay(2);
        Control = AT_READ_REG(hw, REG_VPD_CAP);
        if (Control & VPD_CAP_VPD_FLAG)
            break;
    }
    if (Control & VPD_CAP_VPD_FLAG)
    {
        *pValue = AT_READ_REG(hw, REG_VPD_DATA);
        return true;
    }
    return false; // timeout
}

void at_force_ps(at_hw* hw)
{
	AT_WRITE_REGW(hw, 
		REG_GPHY_CTRL, 
		GPHY_CTRL_PW_WOL_DIS|GPHY_CTRL_EXT_RESET);
}




