/*
 * Copyright (c) 2014, Manuel Vetterli
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */ 

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>

#include "config.h"
#include "eeprom.h"

t_eeprom_storage eeprom;
t_eeprom_storage_aligned eeprom_shadow;
t_eeprom_storage_aligned eeprom_eemem[EEPROM_STORAGE_ENTRY_COUNT] EEMEM;
t_eeprom_status eeprom_status;
t_eeprom_status_aligned eeprom_status_shadow;
t_eeprom_status_aligned eeprom_status_eemem[EEPROM_STATUS_ENTRY_COUNT] EEMEM;

const t_eeprom_default eeprom_default PROGMEM = {
	.eeprom = {
				.salt = SOFTWARE_VERSION,
				.sv_serial_number = 0xFFFF,
				.sv_destination_id = 0xFFFF,
	#define EEPROM_DEFAULT
	#include "config.h"
	#undef EEPROM_DEFAULT
			},

	.eeprom_status = {
		.flags = 0,
	#define EEPROM_STATUS_DEFAULT
	#include "config.h"
	#undef EEPROM_STATUS_DEFAULT
		}
};

void eeprom_load_status(void)
{
	uint8_t index;
	int8_t best_index = -1;
	uint8_t best_version = 0xFF;
	
	for (index=0;index<EEPROM_STATUS_ENTRY_COUNT;index++)
	{
		// Read info header
		eeprom_read_block(&eeprom_status_shadow.info, &eeprom_status_eemem[index].info, sizeof(t_eeprom_info));
		
		// Check if there is any data
		if (eeprom_status_shadow.info.magic!=EEPROM_MAGIC_BYTE)
		{
			continue;
		}
		
		// Read complete eeprom structure
		eeprom_read_block(&eeprom_status_shadow.eeprom, &eeprom_status_eemem[index].eeprom, sizeof(t_eeprom_status));
		
		// Calculate checksum and compare to stored one
		uint16_t checksum = eeprom_status_checksum();
		if (eeprom_status_shadow.info.checksum!=checksum)
		{
			continue;
		}
		
		// First index loaded
		if (best_index<0)
		{
			best_index = index;
			best_version = eeprom_status_shadow.info.version;
		}
		else
		{
			// Check if version is newer than best_version
			if (((eeprom_status_shadow.info.version < best_version) && (best_version - eeprom_status_shadow.info.version) > 128)
			|| ((eeprom_status_shadow.info.version > best_version) && (eeprom_status_shadow.info.version - best_version) < 128))
			{
				best_index = index;
				best_version = eeprom_status_shadow.info.version;
			}
		}
	}
	
	if (best_index<0)
	{
		// No valid config found --> load default
		memcpy_P(&eeprom_status_shadow.eeprom, &eeprom_default.eeprom_status, sizeof(t_eeprom_status));
		eeprom_status_shadow.info.checksum = eeprom_status_checksum();
		eeprom_status_shadow.info.magic = EEPROM_MAGIC_BYTE;
		eeprom_status_shadow.info.version = 0;
		eeprom_update_block(&eeprom_status_shadow, &eeprom_status_eemem[0], sizeof(t_eeprom_status_aligned));
		memcpy(&eeprom_status, &eeprom_status_shadow.eeprom, sizeof(t_eeprom_status));
		eeprom_status_shadow.info.magic = 0;
	}
	else
	{
		// Load best config into RAM
		eeprom_read_block(&eeprom_status_shadow, &eeprom_status_eemem[best_index], sizeof(t_eeprom_status_aligned));
		memcpy(&eeprom_status, &eeprom_status_shadow.eeprom, sizeof(t_eeprom_status));
		eeprom_status_shadow.info.magic = best_index;
	}
}

void eeprom_load_storage(void)
{
	uint8_t index;
	int8_t best_index = -1;
	uint8_t best_version = 0xFF;
	
	for (index=0;index<EEPROM_STORAGE_ENTRY_COUNT;index++)
	{
		// Read info header
		eeprom_read_block(&eeprom_shadow.info, &eeprom_eemem[index].info, sizeof(t_eeprom_info));
		
		// Check if there is any data
		if (eeprom_shadow.info.magic!=EEPROM_MAGIC_BYTE)
		{
			continue;
		}
		
		// Read complete eeprom structure
		eeprom_read_block(&eeprom_shadow.eeprom, &eeprom_eemem[index].eeprom, sizeof(t_eeprom_storage));
		
		// Calculate checksum and compare to stored one
		uint16_t checksum = eeprom_checksum();
		if (eeprom_shadow.info.checksum!=checksum)
		{
			continue;
		}
		
		// First index loaded
		if (best_index<0)
		{
			best_index = index;
			best_version = eeprom_shadow.info.version;
		}
		else
		{
			// Check if version is newer than best_version
			if (((eeprom_shadow.info.version < best_version) && (best_version - eeprom_shadow.info.version) > 128)
			|| ((eeprom_shadow.info.version > best_version) && (eeprom_shadow.info.version - best_version) < 128))
			{
				best_index = index;
				best_version = eeprom_shadow.info.version;
			}
		}
	}
	
	if (best_index<0)
	{
		// No valid config found --> load default
		memcpy_P(&eeprom_shadow.eeprom, &eeprom_default.eeprom, sizeof(t_eeprom_storage));
		eeprom_shadow.info.checksum = eeprom_checksum();
		eeprom_shadow.info.magic = EEPROM_MAGIC_BYTE;
		eeprom_shadow.info.version = 0;
		eeprom_update_block(&eeprom_shadow, &eeprom_eemem[0], sizeof(t_eeprom_storage_aligned));
		memcpy(&eeprom, &eeprom_shadow.eeprom, sizeof(t_eeprom_storage));
		eeprom_shadow.info.magic = 0;
	}
	else
	{
		// Load best config into RAM
		eeprom_read_block(&eeprom_shadow, &eeprom_eemem[best_index], sizeof(t_eeprom_storage_aligned));
		memcpy(&eeprom, &eeprom_shadow.eeprom, sizeof(t_eeprom_storage));
		eeprom_shadow.info.magic = best_index;
	}
}

void eeprom_load_defaults(void)
{
	memcpy_P(&eeprom, &eeprom_default.eeprom, sizeof(t_eeprom_storage));
	memcpy_P(&eeprom_status, &eeprom_default.eeprom_status, sizeof(t_eeprom_status));
}

void eeprom_sync_storage(void)
{
	uint8_t partition_index;
	
    if (memcmp(&eeprom, &eeprom_shadow.eeprom, sizeof(t_eeprom_storage)))
    {
		memcpy(&eeprom_shadow.eeprom, &eeprom, sizeof(t_eeprom_storage));
		partition_index = (eeprom_shadow.info.magic + 1) % EEPROM_STORAGE_ENTRY_COUNT;			
		eeprom_shadow.info.magic = EEPROM_MAGIC_BYTE;
		eeprom_shadow.info.version++;
		eeprom_shadow.info.checksum = eeprom_checksum();
		eeprom_update_block(&eeprom_shadow, &eeprom_eemem[partition_index], sizeof(t_eeprom_storage_aligned));
		eeprom_shadow.info.magic = partition_index;
    }
}

void eeprom_sync_status(void)
{  
	uint8_t partition_index;
	
	if (memcmp(&eeprom_status, &eeprom_status_shadow.eeprom, sizeof(t_eeprom_status)))
	{
		memcpy(&eeprom_status_shadow.eeprom, &eeprom_status, sizeof(t_eeprom_status));
		partition_index = (eeprom_status_shadow.info.magic + 1) % EEPROM_STATUS_ENTRY_COUNT;
		eeprom_status_shadow.info.magic = EEPROM_MAGIC_BYTE;
		eeprom_status_shadow.info.version++;
		eeprom_status_shadow.info.checksum = eeprom_status_checksum();
		eeprom_update_block(&eeprom_status_shadow, &eeprom_status_eemem[partition_index], sizeof(t_eeprom_status_aligned));
		eeprom_status_shadow.info.magic = partition_index;
	}
}

uint16_t eeprom_checksum(void)
{
	uint16_t crc = 0;
	uint16_t index = 0;
	
	for (index=0;index<sizeof(t_eeprom_storage);index++)
	{
		crc = _crc_xmodem_update(crc, ((uint8_t *) &eeprom_shadow.eeprom)[index]);	
	}
	
	return crc;
}

uint16_t eeprom_status_checksum(void)
{
	uint16_t crc = 0;
	uint16_t index = 0;
	
	for (index=0;index<sizeof(t_eeprom_status);index++)
	{
		crc = _crc_xmodem_update(crc, ((uint8_t *) &eeprom_status_shadow.eeprom)[index]);
	}
	
	return crc;
}
