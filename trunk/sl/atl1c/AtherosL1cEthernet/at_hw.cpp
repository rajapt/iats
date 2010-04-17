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

#include <IOKit/pci/IOPCIDevice.h>
#include "at_hw.h"
#include "at.h"
#include "mii.h"

#define LBYTESWAP( a )  ( ( ( (a) & 0x00ff00ff ) << 8 ) | ( ( (a) & 0xff00ff00 ) >> 8 ) )
#define LONGSWAP( a )   ( ( LBYTESWAP( a ) << 16 ) | ( LBYTESWAP( a ) >> 16 ) )
#define SHORTSWAP( a )  ( ( (a) << 8 ) | ( (a) >> 8 ) )

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
/*
 * check_eeprom_exist
 * return 1 if eeprom exist
 */
int atl1c_check_eeprom_exist(atl1c_hw *hw)
{
	u32 data;
	
	AT_READ_REG(hw, REG_TWSI_DEBUG, &data);
	if (data & TWSI_DEBUG_DEV_EXIST)
		return 1;
	
	AT_READ_REG(hw, REG_MASTER_CTRL, &data);
	if (data & MASTER_CTRL_OTP_SEL)
		return 1;
	return 0;
}

void atl1c_hw_set_mac_addr(atl1c_hw *hw)
{
	u32 value;
	/*
	 * 00-0B-6A-F6-00-DC
	 * 0:  6AF600DC 1: 000B
	 * low dword
	 */
	value = (((u32)hw->mac_addr[2]) << 24) |
	(((u32)hw->mac_addr[3]) << 16) |
	(((u32)hw->mac_addr[4]) << 8)  |
	(((u32)hw->mac_addr[5])) ;
	AT_WRITE_REG_ARRAY(hw, REG_MAC_STA_ADDR, 0, value);
	/* hight dword */
	value = (((u32)hw->mac_addr[0]) << 8) |
	(((u32)hw->mac_addr[1])) ;
	AT_WRITE_REG_ARRAY(hw, REG_MAC_STA_ADDR, 1, value);
}

/*
 * atl1c_get_permanent_address
 * return 0 if get valid mac address,
 */
int atl1c_get_permanent_address(atl1c_hw *hw)
{
	u32 addr[2];
	u32 i;
	u32 otp_ctrl_data;
	u32 twsi_ctrl_data;
	u32 ltssm_ctrl_data;
	u32 wol_data;
	u8  eth_addr[ETH_ALEN];
	u16 phy_data;
	u32 raise_vol = 0;
	
	
	/* init */
	addr[0] = addr[1] = 0;
	AT_READ_REG(hw, REG_OTP_CTRL, &otp_ctrl_data);
	if (atl1c_check_eeprom_exist(hw)) {
		/* Enable OTP CLK */
		if (hw->nic_type == athr_l1c || hw->nic_type == athr_l2c_b) {
			if (!(otp_ctrl_data & OTP_CTRL_CLK_EN)) {
				otp_ctrl_data |= OTP_CTRL_CLK_EN;
				AT_WRITE_REG(hw, REG_OTP_CTRL, otp_ctrl_data);
				msec_delay(1);
			}
		}
		if (hw->nic_type == athr_l2c_b || hw->nic_type == athr_l2c_b2 ||
			hw->nic_type == athr_l1d) {
			atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x00);
			if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data) != 0)
				goto out;
			phy_data &= 0xFF7F;
			atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data);
			
			atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x3B);
			if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data) != 0)
				goto out;
			phy_data |= 0x8;
			atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data);
			usec_delay(20);
			raise_vol = 1;	
		}
		/* close open bit of ReadOnly*/
		AT_READ_REG(hw, REG_LTSSM_ID_CTRL, &ltssm_ctrl_data);
		ltssm_ctrl_data &= ~LTSSM_ID_EN_WRO;
		AT_WRITE_REG(hw, REG_LTSSM_ID_CTRL, ltssm_ctrl_data);
		
		/* clear any WOL settings */
		AT_WRITE_REG(hw, REG_PM_CTRLSTAT, 0);
		AT_WRITE_REG(hw, REG_WOL_CTRL, 0);
		AT_READ_REG(hw, REG_WOL_CTRL, &wol_data);
		
		
		AT_READ_REG(hw, REG_TWSI_CTRL, &twsi_ctrl_data);
		twsi_ctrl_data |= TWSI_CTRL_SW_LDSTART;
		AT_WRITE_REG(hw, REG_TWSI_CTRL, twsi_ctrl_data);
		for (i = 0; i < AT_TWSI_EEPROM_TIMEOUT; i++) {
			msec_delay(10);
			AT_READ_REG(hw, REG_TWSI_CTRL, &twsi_ctrl_data);
			if ((twsi_ctrl_data & TWSI_CTRL_SW_LDSTART) == 0)
				break;
		}
		if (i >= AT_TWSI_EEPROM_TIMEOUT)
			return AT_ERR_TIMEOUT;
	}
	/* Disable OTP_CLK */	
	if ((hw->nic_type == athr_l1c || hw->nic_type == athr_l2c)) {
		otp_ctrl_data &= ~OTP_CTRL_CLK_EN;
		AT_WRITE_REG(hw, REG_OTP_CTRL, otp_ctrl_data);
		msec_delay(1);
	}
	if (raise_vol) {
		if (hw->nic_type == athr_l2c_b || hw->nic_type == athr_l2c_b2 ||
			hw->nic_type == athr_l1d) {
			atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x00);
			if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data) != 0)
				goto out;
			phy_data |= 0x80;
			atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data);
			
			atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x3B);
			if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data) != 0)
				goto out;
			phy_data &= 0xFFF7;
			atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data);
			usec_delay(20);
		}
		
	}	
	/* maybe MAC-address is from BIOS */
	AT_READ_REG(hw, REG_MAC_STA_ADDR, &addr[0]);
	AT_READ_REG(hw, REG_MAC_STA_ADDR + 4, &addr[1]);
	*(u32 *) &eth_addr[2] = LONGSWAP(addr[0]);
	*(u16 *) &eth_addr[0] = SHORTSWAP(*(u16 *)&addr[1]);
	
	if (is_valid_ether_addr(eth_addr)) {
		memcpy(hw->perm_mac_addr, eth_addr, ETH_ALEN);
		return 0;
	}
out:
	return AT_ERR_EEPROM;
}

bool atl1c_write_eeprom(atl1c_hw *hw, u32 offset, u32 value)
{
	return true;
}

bool atl1c_read_eeprom(atl1c_hw *hw, u32 offset, u32 *p_value)
{
	int i;
	int ret = false;
	u32 otp_ctrl_data;
	u32 control;
	u32 data;
	
	if (offset & 3)
		return ret; /* address do not align */
	
	AT_READ_REG(hw, REG_OTP_CTRL, &otp_ctrl_data);
	if (!(otp_ctrl_data & OTP_CTRL_CLK_EN))
		AT_WRITE_REG(hw, REG_OTP_CTRL, 
					 (otp_ctrl_data | OTP_CTRL_CLK_EN));
	
	AT_WRITE_REG(hw, REG_EEPROM_DATA_LO, 0);
	control = (offset & EEPROM_CTRL_ADDR_MASK) << EEPROM_CTRL_ADDR_SHIFT;
	AT_WRITE_REG(hw, REG_EEPROM_CTRL, control);
	
	for (i = 0; i < 10; i++) {
		usec_delay(100);
		AT_READ_REG(hw, REG_EEPROM_CTRL, &control);
		if (control & EEPROM_CTRL_RW)
			break;
	}
	if (control & EEPROM_CTRL_RW) {
		AT_READ_REG(hw, REG_EEPROM_CTRL, &data);
		AT_READ_REG(hw, REG_EEPROM_DATA_LO, p_value);
		data = data & 0xFFFF;
		*p_value = LONGSWAP((data << 16) | (*p_value >> 16));
		ret = true;
	}
	if (!(otp_ctrl_data & OTP_CTRL_CLK_EN))
		AT_WRITE_REG(hw, REG_OTP_CTRL, otp_ctrl_data);
	
	return ret;
}
/*
 * Reads the adapter's MAC address from the EEPROM
 *
 * hw - Struct containing variables accessed by shared code
 */
int atl1c_read_mac_addr(atl1c_hw *hw)
{
	int err = 0;
	
	err = atl1c_get_permanent_address(hw);
	if (err) {
        hw->perm_mac_addr[0] = 0x00;
        hw->perm_mac_addr[1] = 0x13;
        hw->perm_mac_addr[2] = 0x74;
        hw->perm_mac_addr[3] = 0x00;
        hw->perm_mac_addr[4] = 0x5c;
        hw->perm_mac_addr[5] = 0x38;
    }
	
	//		return AT_ERR_EEPROM;
	memcpy(hw->mac_addr, hw->perm_mac_addr, sizeof(hw->perm_mac_addr));
	return 0;
}

/*
 * atl1c_hash_mc_addr
 *  purpose
 *      set hash value for a multicast address
 *      hash calcu processing :
 *          1. calcu 32bit CRC for multicast address
 *          2. reverse crc with MSB to LSB
 */
u32 atl1c_hash_mc_addr(atl1c_hw *hw, u8 *mc_addr)
{
	u32 crc32;
	u32 value = 0;
	int i;
	
	crc32 = ether_crc_le(6, mc_addr);
	for (i = 0; i < 32; i++)
		value |= (((crc32 >> i) & 1) << (31 - i));
	
	return value;
}

/*
 * Sets the bit in the multicast table corresponding to the hash value.
 * hw - Struct containing variables accessed by shared code
 * hash_value - Multicast address hash value
 */
void atl1c_hash_set(atl1c_hw *hw, u32 hash_value)
{
	u32 hash_bit, hash_reg;
	u32 mta;
	
	/*
	 * The HASH Table  is a register array of 2 32-bit registers.
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
 * Reads the value from a PHY register
 * hw - Struct containing variables accessed by shared code
 * reg_addr - address of the PHY register to read
 */
int atl1c_read_phy_reg(atl1c_hw *hw, u16 reg_addr, u16 *phy_data)
{
	u32 val;
	int i;
	
	val = ((u32)(reg_addr & MDIO_REG_ADDR_MASK)) << MDIO_REG_ADDR_SHIFT |
	MDIO_START | MDIO_SUP_PREAMBLE | MDIO_RW |
	MDIO_CLK_25_4 << MDIO_CLK_SEL_SHIFT;
	
	AT_WRITE_REG(hw, REG_MDIO_CTRL, val);
	
	//wmb();
	
	for (i = 0; i < MDIO_WAIT_TIMES; i++) {
		usec_delay(2);
		AT_READ_REG(hw, REG_MDIO_CTRL, &val);
		if (!(val & (MDIO_START | MDIO_BUSY)))
			break;
		//wmb();
	}
	if (!(val & (MDIO_START | MDIO_BUSY))) {
		*phy_data = (u16)val;
		return 0;
	}
	
	return AT_ERR_PHY;
}

/*
 * Writes a value to a PHY register
 * hw - Struct containing variables accessed by shared code
 * reg_addr - address of the PHY register to write
 * data - data to write to the PHY
 */
int atl1c_write_phy_reg(atl1c_hw *hw, u32 reg_addr, u16 phy_data)
{
	int i;
	u32 val;
	
	val = ((u32)(phy_data & MDIO_DATA_MASK)) << MDIO_DATA_SHIFT   |
	(reg_addr & MDIO_REG_ADDR_MASK) << MDIO_REG_ADDR_SHIFT |
	MDIO_SUP_PREAMBLE | MDIO_START |
	MDIO_CLK_25_4 << MDIO_CLK_SEL_SHIFT;
	
	AT_WRITE_REG(hw, REG_MDIO_CTRL, val);
	//wmb();
	
	for (i = 0; i < MDIO_WAIT_TIMES; i++) {
		usec_delay(2);
		AT_READ_REG(hw, REG_MDIO_CTRL, &val);
		if (!(val & (MDIO_START | MDIO_BUSY)))
			break;
		//wmb();
	}
	
	if (!(val & (MDIO_START | MDIO_BUSY)))
		return 0;
	
	return AT_ERR_PHY;
}

/*
 * Configures PHY autoneg and flow control advertisement settings
 *
 * hw - Struct containing variables accessed by shared code
 */
static int atl1c_phy_setup_adv(atl1c_hw *hw)
{
	u16 mii_adv_data = ADVERTISE_DEFAULT_CAP & ~ADVERTISE_SPEED_MASK;
	u16 mii_giga_ctrl_data = GIGA_CR_1000T_DEFAULT_CAP &
	~GIGA_CR_1000T_SPEED_MASK;
	u16 tmp_data;
	
	if (hw->autoneg_advertised & ADVERTISE_10_HALF)
		mii_adv_data |= ADVERTISE_10T_HD_CAPS;
	if (hw->autoneg_advertised & ADVERTISE_10_FULL)
		mii_adv_data |= ADVERTISE_10T_FD_CAPS;
	if (hw->autoneg_advertised & ADVERTISE_100_HALF)
		mii_adv_data |= ADVERTISE_100TX_HD_CAPS;
	if (hw->autoneg_advertised & ADVERTISE_100_FULL)
		mii_adv_data |= ADVERTISE_100TX_FD_CAPS;
	if (hw->link_cap_flags & LINK_CAP_SPEED_1000) {
		if (hw->autoneg_advertised & ADVERTISE_1000_HALF)
			mii_giga_ctrl_data |= GIGA_CR_1000T_HD_CAPS;
		if (hw->autoneg_advertised & ADVERTISE_1000_FULL)
			mii_giga_ctrl_data |= GIGA_CR_1000T_FD_CAPS;
	}
	if (hw->nic_type == athr_l2c_b2) {
		atl1c_read_phy_reg(hw, MII_BMSR, &tmp_data);
		tmp_data |= 0x02;
		atl1c_write_phy_reg(hw, 0x3C, tmp_data);
	}	
	if (atl1c_write_phy_reg(hw, MII_ADVERTISE, mii_adv_data) != 0 ||
	    atl1c_write_phy_reg(hw, MII_GIGA_CR, mii_giga_ctrl_data) != 0)
		return AT_ERR_PHY_SPEED;
	return 0;
}


void atl1c_phy_disable(atl1c_hw *hw)
{
	AT_WRITE_REGW(hw, REG_GPHY_CTRL,
				  GPHY_CTRL_PW_WOL_DIS | GPHY_CTRL_EXT_RESET);
}

static void atl1c_phy_magic_data(atl1c_hw *hw)
{
	u16 data;
	
	atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x12);
	atl1c_write_phy_reg(hw, MII_DBG_DATA, 0x4C04);
	
	atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x05);
	atl1c_write_phy_reg(hw, MII_DBG_DATA, 0x2C46);
	
	atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x36);
	atl1c_write_phy_reg(hw, MII_DBG_DATA, 0xE12C);
	
	atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x04);
	atl1c_write_phy_reg(hw, MII_DBG_DATA, 0x88BB);
	
	atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x00);
	atl1c_write_phy_reg(hw, MII_DBG_DATA, 0x02EF);
	
	atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x36);
	if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &data) != 0)
		return;
	data |= 0x80;
	atl1c_write_phy_reg(hw, MII_DBG_DATA, data);
	
	if (hw->ctrl_flags & ATL1C_HIB_DISABLE) {
		atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x29);
		if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &data) != 0)
			return;
		data &= 0x7FFF;
		atl1c_write_phy_reg(hw, MII_DBG_DATA, data);
		
		atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0xB);
		if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &data) != 0)
			return;
		data &= 0x7FFF;
		atl1c_write_phy_reg(hw, MII_DBG_DATA, data);
	}
}	

int atl1c_phy_reset(atl1c_hw *hw)
{
	u16 phy_data;
	u32 phy_ctrl_data = GPHY_CTRL_DEFAULT;
	u32 mii_ier_data = IER_LINK_UP | IER_LINK_DOWN;
	int err;
	
	if (hw->ctrl_flags & ATL1C_HIB_DISABLE)
		phy_ctrl_data &= ~GPHY_CTRL_HIB_EN;
	
	AT_WRITE_REG(hw, REG_GPHY_CTRL, phy_ctrl_data);
	AT_WRITE_FLUSH(hw);
	msec_delay(40);
	phy_ctrl_data |= GPHY_CTRL_EXT_RESET;
	AT_WRITE_REG(hw, REG_GPHY_CTRL, phy_ctrl_data);
	AT_WRITE_FLUSH(hw);
	msec_delay(10);
	
	if (hw->nic_type == athr_l2c_b) {
		atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x0A);
		atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data);
		atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data & 0xDFFF);
		
	}
	if (hw->nic_type == athr_l2c_b || hw->nic_type == athr_l2c_b2 ||
		hw->nic_type == athr_l1d) {	
		atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x3B);
		atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data);
		atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data & 0xFFF7);
		msec_delay(20);
	}
	if (hw->nic_type == athr_l1d) {
		atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x29);
		atl1c_write_phy_reg(hw, MII_DBG_DATA, 0x929D);
	}
	if (hw->nic_type == athr_l1c || hw->nic_type == athr_l2c_b2
		|| hw->nic_type == athr_l2c || hw->nic_type == athr_l2c) {
		atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x29);
		atl1c_write_phy_reg(hw, MII_DBG_DATA, 0xB6DD);
	}
	err = atl1c_write_phy_reg(hw, MII_IER, mii_ier_data);
	if (err) {
		AT_ERR("Error enable PHY linkChange Interrupt\n");
		return err;
	}
	if (!(hw->ctrl_flags & ATL1C_FPGA_VERSION))
		atl1c_phy_magic_data(hw);
	return 0;
}

int atl1c_phy_init(atl1c_hw *hw)
{
	int ret_val;
	u16 mii_bmcr_data = BMCR_RESET;
	
	if ((atl1c_read_phy_reg(hw, MII_PHYSID1, &hw->phy_id1) != 0) ||
		(atl1c_read_phy_reg(hw, MII_PHYSID2, &hw->phy_id2) != 0)) {
		AT_ERR("Error get phy ID\n");
		return -1;
	}
	switch (hw->media_type) {
		case MEDIA_TYPE_AUTO_SENSOR:
		case MEDIA_TYPE_1000M_FULL:
			ret_val = atl1c_phy_setup_adv(hw);
			if (ret_val) {
				AT_ERR(
						"Error Setting up Auto-Negotiation\n");
				return ret_val;
			}
			mii_bmcr_data |= BMCR_AUTO_NEG_EN | BMCR_RESTART_AUTO_NEG;
			break;
		case MEDIA_TYPE_100M_FULL:
			mii_bmcr_data |= BMCR_SPEED_100 | BMCR_FULL_DUPLEX;
			break;
		case MEDIA_TYPE_100M_HALF:
			mii_bmcr_data |= BMCR_SPEED_100;
			break;
		case MEDIA_TYPE_10M_FULL:
			mii_bmcr_data |= BMCR_SPEED_10 | BMCR_FULL_DUPLEX;
			break;
		case MEDIA_TYPE_10M_HALF:
			mii_bmcr_data |= BMCR_SPEED_10;
			break;
		default:
			AT_ERR("Wrong Media type %d\n", hw->media_type);
			return AT_ERR_PARAM;
			break;
	}
	
	ret_val = atl1c_write_phy_reg(hw, MII_BMCR, mii_bmcr_data);
	if (ret_val)
		return ret_val;
	hw->phy_configured = true;
	
	return 0;
}

/*
 * Detects the current speed and duplex settings of the hardware.
 *
 * hw - Struct containing variables accessed by shared code
 * speed - Speed of the connection
 * duplex - Duplex setting of the connection
 */
int atl1c_get_speed_and_duplex(atl1c_hw *hw, u16 *speed, u16 *duplex)
{
	int err;
	u16 phy_data;
	
	/* Read   PHY Specific Status Register (17) */
	err = atl1c_read_phy_reg(hw, MII_GIGA_PSSR, &phy_data);
	if (err)
		return err;
	
	if (!(phy_data & GIGA_PSSR_SPD_DPLX_RESOLVED))
		return AT_ERR_PHY_RES;
	
	switch (phy_data & GIGA_PSSR_SPEED) {
		case GIGA_PSSR_1000MBS:
			*speed = SPEED_1000;
			break;
		case GIGA_PSSR_100MBS:
			*speed = SPEED_100;
			break;
		case  GIGA_PSSR_10MBS:
			*speed = SPEED_10;
			break;
		default:
			return AT_ERR_PHY_SPEED;
			break;
	}
	
	if (phy_data & GIGA_PSSR_DPLX)
		*duplex = FULL_DUPLEX;
	else
		*duplex = HALF_DUPLEX;
	
	return 0;
}

int atl1c_phy_power_saving(atl1c_hw * hw)
{
	struct atl1c_adapter *adapter = (struct atl1c_adapter *)hw->adapter;
	int ret = 0;
	u16 autoneg_advertised = ADVERTISE_10_HALF; 
	u16 save_autoneg_advertised;
	u16 phy_data;
	u16 mii_lpa_data;
	u16 speed = SPEED_0;
	u16 duplex = FULL_DUPLEX;
	int i;
	
	atl1c_read_phy_reg(hw, MII_LPA, &mii_lpa_data);
	if (mii_lpa_data & LPA_10FULL)
		autoneg_advertised = ADVERTISE_10_FULL;
	else if (mii_lpa_data & LPA_10HALF)
		autoneg_advertised = ADVERTISE_10_HALF;
	else if (mii_lpa_data & LPA_100HALF)
		autoneg_advertised = ADVERTISE_100_HALF;
	else if (mii_lpa_data & LPA_100FULL)
		autoneg_advertised = ADVERTISE_100_FULL;
	
	save_autoneg_advertised = hw->autoneg_advertised;
	hw->phy_configured = false;
	hw->autoneg_advertised = autoneg_advertised;
	if (atl1c_restart_autoneg(hw) != 0) {
		DEBUGOUT( "phy autoneg failed\n");
		ret = -1;
	}
	hw->autoneg_advertised = save_autoneg_advertised;
	
	if (mii_lpa_data) {
		for (i = 0; i < AT_SUSPEND_LINK_TIMEOUT; i++) {
			msec_delay(100);
			atl1c_read_phy_reg(hw, MII_BMSR, &phy_data);
			atl1c_read_phy_reg(hw, MII_BMSR, &phy_data);
			if (phy_data & MII_BMSR) {
				if (atl1c_get_speed_and_duplex(hw, &speed,
											   &duplex) != 0)
					DEBUGOUT(
							"get speed and duplex failed\n");
			}
		}
	}
	adapter->link_speed = speed;
	adapter->link_duplex = duplex;
	
	return ret;
}

int atl1c_restart_autoneg(struct atl1c_hw *hw)
{
	int err = 0;
	u16 mii_bmcr_data = BMCR_RESET;
	
	err = atl1c_phy_setup_adv(hw);
	if (err)
		return err;
	mii_bmcr_data |= BMCR_AUTO_NEG_EN | BMCR_RESTART_AUTO_NEG;
	
	return atl1c_write_phy_reg(hw, MII_BMCR, mii_bmcr_data);
}

