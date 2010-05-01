/*
 *  at_param.h-- ATL2 user options definitions
 *
 * Copyright (c) 2009 maolj <maolj@hotmail.com>.
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
 * Driver for Atheros L2 Fast Ethernet adapter.  This 10/100 Mbit NIC
 * is included in many ASUS products, including the ASUS EeePC.
 *
 * This driver is heavily based on ATL2 Linux driver by xiong huang <xiong.huang@atheros.com>.
 */

#ifndef _AT_PARAM_H__
#define _AT_PARAM_H__

void at_check_options(at_adapter *adapter);

#endif//_AT_PARAM_H__