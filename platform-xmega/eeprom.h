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

#ifndef EEPROM_H
#define	EEPROM_H

#ifdef	__cplusplus
extern "C" {
#endif

#define EEPROM_MAGIC_BYTE	0xAA
#define EEPROM_STORAGE_ENTRY_COUNT	2UL
#define EEPROM_STATUS_ENTRY_COUNT	((EEPROM_SIZE-(EEPROM_STORAGE_ENTRY_COUNT*sizeof (t_eeprom_storage_aligned)))/sizeof (t_eeprom_status_aligned))

typedef struct {
	uint8_t salt;
	uint16_t sv_serial_number;
	uint16_t sv_destination_id;
#define EEPROM_CFG
#include "config.h"
#undef EEPROM_CFG
} t_eeprom_storage;

typedef struct {
	uint8_t flags;
#define EEPROM_STATUS_CFG
#include "config.h"
#undef EEPROM_STATUS_CFG
} t_eeprom_status;

typedef struct {
	t_eeprom_storage eeprom;
	t_eeprom_status eeprom_status;
} t_eeprom_default;

typedef struct {
	uint8_t magic;
	uint8_t version;
	uint16_t checksum;
} t_eeprom_info;

typedef struct {
	t_eeprom_info info;
	t_eeprom_storage eeprom;
	char _padding[EEPROM_PAGE_SIZE * ((sizeof (t_eeprom_storage) + sizeof (t_eeprom_info) + EEPROM_PAGE_SIZE - 1) / EEPROM_PAGE_SIZE) - sizeof (t_eeprom_storage) - sizeof (t_eeprom_info)];
} t_eeprom_storage_aligned;

typedef struct {
	t_eeprom_info info;
	t_eeprom_status eeprom;
	char _padding[EEPROM_PAGE_SIZE * ((sizeof (t_eeprom_status) + sizeof (t_eeprom_info) + EEPROM_PAGE_SIZE - 1) / EEPROM_PAGE_SIZE) - sizeof (t_eeprom_status) - sizeof (t_eeprom_info)];
} t_eeprom_status_aligned;

uint16_t eeprom_checksum(void);
uint16_t eeprom_status_checksum(void);
void eeprom_load_storage(void);
void eeprom_sync_storage(void);
void eeprom_load_status(void);
void eeprom_sync_status(void);
void eeprom_load_defaults(void);

extern t_eeprom_storage eeprom;
extern t_eeprom_storage_aligned eeprom_shadow;
extern t_eeprom_status eeprom_status;
extern t_eeprom_status_aligned eeprom_status_shadow;
extern t_eeprom_storage_aligned eeprom_eemem[];
extern t_eeprom_status_aligned eeprom_status_eemem[];

#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_H */

