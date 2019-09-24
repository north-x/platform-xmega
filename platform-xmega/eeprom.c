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
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>

#include "config.h"
#include "eeprom.h"

t_eeprom_storage_aligned eeprom;
t_eeprom_storage_aligned eeprom_eemem[EEPROM_STORAGE_ENTRY_COUNT] __attribute__((section(".eeprom")));
t_eeprom_status_aligned eeprom_status;
t_eeprom_status_aligned eeprom_status_eemem[EEPROM_STATUS_ENTRY_COUNT] __attribute__((section(".eeprom")));

const t_eeprom_default eeprom_default PROGMEM = {
	.eeprom = {
				.version = SOFTWARE_VERSION,
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

void eeprom_init(void)
{
	while (NVM.STATUS&NVM_NVMBUSY_bm);
	
	NVM.CTRLB |= NVM_EEMAPEN_bm;
	
	eeprom_load_status();
	eeprom_load_storage();
}

uint16_t eeprom_checksum(const void *data, uint16_t length)
{
	uint16_t crc = 0;
	size_t index = 0;
	
	for (index=0;index<length;index++)
	{
		crc = _crc_xmodem_update(crc, ((uint8_t *) data)[index]);
	}
	
	return crc;
}

void eeprom_write_block(const void *src, void *dst, uint16_t length)
{
	uint8_t remainder = 0;
	uint16_t srcIndex = (uint16_t) src;
	uint16_t dstIndex = (uint16_t) dst;

	while (NVM.STATUS&NVM_NVMBUSY_bm);
	
	while (length)
	{
		*(uint8_t *)((uint16_t)dstIndex++ + MAPPED_EEPROM_START) = *(uint8_t*)srcIndex++;
		
		remainder = dstIndex % EEPROM_PAGE_SIZE;

		if (remainder==0)
		{
			eeprom_write_page(dstIndex-1);
			while (NVM.STATUS&NVM_NVMBUSY_bm);
		}

		length--;
	}

	if (remainder!=0)
	{
		eeprom_write_page(dstIndex-1);
	}	
}

inline void eeprom_write_page(uint16_t addr)
{
	while (NVM.STATUS&NVM_NVMBUSY_bm);
	
	NVM.ADDR0 = addr&0xFF;
	NVM.ADDR1 = (addr>>8)&0x1F;
	NVM.ADDR2 = 0x00;

	NVM.CMD   = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;

	CCP = CCP_IOREG_gc;
	NVM.CTRLA = NVM_CMDEX_bm;
}

inline void * eeprom_read_block(void *dst, void *src, uint16_t length)
{
	return memcpy(dst, (uint8_t *)(uint16_t)src + MAPPED_EEPROM_START, length);
}

void eeprom_load_status(void)
{
	uint8_t index;
	int8_t best_index = -1;
	uint8_t best_version = 0xFF;
	
	while (NVM.STATUS&NVM_NVMBUSY_bm);
	
	for (index=0;index<EEPROM_STATUS_ENTRY_COUNT;index++)
	{
		// Read info header
		eeprom_read_block(&eeprom_status.info, &eeprom_status_eemem[index].info, sizeof(t_eeprom_info));
		
		// Check if there is any data
		if (eeprom_status.info.magic!=EEPROM_MAGIC_BYTE)
		{
			continue;
		}
		
		// Read complete eeprom structure
		eeprom_read_block(&eeprom_status.data, &eeprom_status_eemem[index].data, sizeof(t_eeprom_status));
		
		// Calculate checksum and compare to stored one
		uint16_t checksum = eeprom_checksum(&eeprom_status.data, sizeof(t_eeprom_status));
		if (eeprom_status.info.checksum!=checksum)
		{
			continue;
		}
		
		// First index loaded
		if (best_index<0)
		{
			best_index = index;
			best_version = eeprom_status.info.version;
		}
		else
		{
			// Check if version is newer than best_version
			if (((eeprom_status.info.version < best_version) && (best_version - eeprom_status.info.version) > 128)
			|| ((eeprom_status.info.version > best_version) && (eeprom_status.info.version - best_version) < 128))
			{
				best_index = index;
				best_version = eeprom_status.info.version;
			}
		}
	}
	
	if (best_index<0)
	{
		// No valid config found --> load default
		memcpy_P(&eeprom_status.data, &eeprom_default.eeprom_status, sizeof(t_eeprom_status));
		eeprom_status.info.checksum = eeprom_checksum(&eeprom_status.data, sizeof(t_eeprom_status));
		eeprom_status.info.magic = EEPROM_MAGIC_BYTE;
		eeprom_status.info.version = 0;
		eeprom_write_block(&eeprom_status, &eeprom_status_eemem[0], sizeof(t_eeprom_status_aligned));
		eeprom_status.info.magic = 0;
	}
	else
	{
		// Load best config into RAM
		eeprom_read_block(&eeprom_status, &eeprom_status_eemem[best_index], sizeof(t_eeprom_status_aligned));
		eeprom_status.info.magic = best_index;
	}
}

void eeprom_load_storage(void)
{
	uint8_t index;
	int8_t best_index = -1;
	uint8_t best_version = 0xFF;
	
	while (NVM.STATUS&NVM_NVMBUSY_bm);
	
	for (index=0;index<EEPROM_STORAGE_ENTRY_COUNT;index++)
	{
		// Read info header
		eeprom_read_block(&eeprom.info, &eeprom_eemem[index].info, sizeof(t_eeprom_info));
		
		// Check if there is any data
		if (eeprom.info.magic!=EEPROM_MAGIC_BYTE)
		{
			continue;
		}
		
		// Read complete eeprom structure
		eeprom_read_block(&eeprom.data, &eeprom_eemem[index].data, sizeof(t_eeprom_storage));
		
		// Calculate checksum and compare to stored one
		uint16_t checksum = eeprom_checksum(&eeprom.data, sizeof(t_eeprom_storage));
		if (eeprom.info.checksum!=checksum)
		{
			continue;
		}
		
		// First index loaded
		if (best_index<0)
		{
			best_index = index;
			best_version = eeprom.info.version;
		}
		else
		{
			// Check if version is newer than best_version
			if (((eeprom.info.version < best_version) && (best_version - eeprom.info.version) > 128)
			|| ((eeprom.info.version > best_version) && (eeprom.info.version - best_version) < 128))
			{
				best_index = index;
				best_version = eeprom.info.version;
			}
		}
	}
	
	if (best_index<0)
	{
		// No valid config found --> load default
		memcpy_P(&eeprom.data, &eeprom_default.eeprom, sizeof(t_eeprom_storage));
		eeprom.info.checksum = eeprom_checksum(&eeprom.data, sizeof(t_eeprom_storage));
		eeprom.info.magic = EEPROM_MAGIC_BYTE;
		eeprom.info.version = 0;
		eeprom_write_block(&eeprom, &eeprom_eemem[0], sizeof(t_eeprom_storage_aligned));
		eeprom.info.magic = 0;
	}
	else
	{
		// Load best config into RAM
		eeprom_read_block(&eeprom, &eeprom_eemem[best_index], sizeof(t_eeprom_storage_aligned));
		eeprom.info.magic = best_index;
	}
}

void eeprom_load_defaults(void)
{
	memcpy_P(&eeprom.data, &eeprom_default.eeprom, sizeof(t_eeprom_storage));
	memcpy_P(&eeprom_status.data, &eeprom_default.eeprom_status, sizeof(t_eeprom_status));
}

void eeprom_sync_storage(void)
{
	uint8_t partition_index;
	uint16_t checksum = eeprom_checksum(&eeprom.data, sizeof(t_eeprom_storage));
	
    if (eeprom.info.checksum!=checksum)
    {
		partition_index = (eeprom.info.magic + 1) % EEPROM_STORAGE_ENTRY_COUNT;			
		eeprom.info.magic = EEPROM_MAGIC_BYTE;
		eeprom.info.version++;
		eeprom.info.checksum = checksum;
		eeprom_write_block(&eeprom, &eeprom_eemem[partition_index], sizeof(t_eeprom_storage_aligned));
		eeprom.info.magic = partition_index;
    }
}

void eeprom_sync_status(void)
{  
	uint8_t partition_index;
	uint16_t checksum = eeprom_checksum(&eeprom_status.data, sizeof(t_eeprom_status));
	
	if (eeprom_status.info.checksum!=checksum)
	{
		partition_index = (eeprom_status.info.magic + 1) % EEPROM_STATUS_ENTRY_COUNT;
		eeprom_status.info.magic = EEPROM_MAGIC_BYTE;
		eeprom_status.info.version++;
		eeprom_status.info.checksum = checksum;
		eeprom_write_block(&eeprom_status, &eeprom_status_eemem[partition_index], sizeof(t_eeprom_status_aligned));
		eeprom_status.info.magic = partition_index;
	}
}
