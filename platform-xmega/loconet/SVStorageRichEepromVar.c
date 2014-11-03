/****************************************************************************
    Copyright (C) 2003,2004 Alex Shepherd, Stefan Bormann

    Portions Copyright (C) Digitrax Inc.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*****************************************************************************

    IMPORTANT:

    Some of the message formats used in this code are Copyright Digitrax, Inc.
    and are used with permission as part of the EmbeddedLocoNet project. That
    permission does not extend to uses in other software products. If you wish
    to use this code, algorithm or these message formats outside of
    EmbeddedLocoNet, please contact Digitrax Inc, for specific permission.

    Note: The sale any LocoNet device hardware (including bare PCB's) that
    uses this or any other LocoNet software, requires testing and certification
    by Digitrax Inc. and will be subject to a licensing agreement.

    Please contact Digitrax Inc. for details.

*****************************************************************************

 Title :   LocoNet SV Configuration module
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
           Stefan Bormann <sbormann71@sourceforge.net>
 Date:     2-Jan-2004
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module implements the API to the EEPROM for SV storage
       for access by sv.c or possibly local application.
	   Compared with SVStoragePlainEeprom.c, this module provides a
	   few more constants via SVs.

*****************************************************************************/


#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include "sysdef.h"		// SOFTWARE_VERSION, SERIAL_NUMBER_*
#include "sv.h"			// SV_EE_SZ_*


#ifndef SOFTWARE_VERSION
#error "you must define 'SOFTWARE_VERSION' in sysdef.h!"
#endif


#ifndef SV_MAX_NUMBER
#error "you must define the highest SV number 'SV_MAX_NUMBER' in sysdef.h"
#endif

#ifdef SV_SIZE_BYTES
#error The #define SV_SIZE_BYTES is outdated and replaced by SV_MAX_NUMBER
#endif


#define USER_BASE_RICH (SV_ADDR_USER_BASE+3)

#define SV_SIZE_BYTES (SV_MAX_NUMBER-USER_BASE_RICH+1)

static uint8_t ucSerialHigh EEMEM;
static uint8_t ucSerialLow  EEMEM;
static uint8_t aucSvStorage[SV_SIZE_BYTES] EEMEM;


byte readSVStorage( word Offset )
{
	switch (Offset)
	{
		case SV_ADDR_SW_VERSION:
								return SOFTWARE_VERSION;

		case SV_ADDR_EEPROM_SIZE:
#if (E2END==0x0FF)	/* E2END is defined in processor include */
								return SV_EE_SZ_256;
#elif (E2END==0x1FF)
								return SV_EE_SZ_512;
#elif (E2END==0x3FF)
								return SV_EE_SZ_1024;
#elif (E2END==0x7FF)
								return SV_EE_SZ_2048;
#elif (E2END==0xFFF)
								return SV_EE_SZ_4096;
#else
								return 0xFF;
#endif

		case SV_ADDR_SERIAL_NUMBER_L:
								return eeprom_read_byte(&ucSerialLow); 

		case SV_ADDR_SERIAL_NUMBER_H:
								return eeprom_read_byte(&ucSerialHigh); 

		case SV_ADDR_USER_BASE+0:
								return HARDWARE_VERSION;

		case SV_ADDR_USER_BASE+1:
								return pgm_read_byte_near(FLASHEND-1);  // HARDWARE_VERSION_FLASH

		case SV_ADDR_USER_BASE+2:
								return pgm_read_byte_near(FLASHEND);    // BOOTLOADER_VERSION[_FLASH]

		case USER_BASE_RICH: // and following
		default:
								if (Offset>SV_MAX_NUMBER)
									return 0;
								else
									return eeprom_read_byte(&aucSvStorage[Offset-USER_BASE_RICH]);
	}
}


byte writeSVStorage( word Offset, byte Value )
{
	switch (Offset)
	{
		case SV_ADDR_SERIAL_NUMBER_L:
			eeprom_write_byte(&ucSerialLow, Value); 
			return 0;

		case SV_ADDR_SERIAL_NUMBER_H:
			eeprom_write_byte(&ucSerialHigh, Value); 
			return 0;

		default:
			if (Offset < USER_BASE_RICH)
				return -1;
			if (Offset > SV_MAX_NUMBER)
				return -1;
			eeprom_write_byte(&aucSvStorage[Offset-USER_BASE_RICH], Value);
			return 0;
	}
}


byte isValidSVStorage(word Offset)
{
	return Offset <= SV_MAX_NUMBER;
}
