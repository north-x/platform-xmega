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

#include "config.h"
#include "eeprom.h"

struct t_eeprom_storage eeprom;
struct t_eeprom_storage eeprom_shadow;
struct t_eeprom_storage eeprom_eemem __attribute__((section (".eeprom,\"aw\",@progbits\n.p2align 5;")));
struct t_eeprom_status eeprom_status;
struct t_eeprom_status eeprom_status_shadow;
struct t_eeprom_status eeprom_status_eemem EEMEM;

const struct t_eeprom_default eeprom_default PROGMEM = {
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
	eeprom_read_block(&eeprom_status, &eeprom_status_eemem, sizeof(t_eeprom_status));
	
	memcpy(&eeprom_status_shadow, &eeprom_status, sizeof(t_eeprom_status));
}

void eeprom_load_storage(void)
{
    eeprom_read_block(&eeprom, &eeprom_eemem, sizeof(t_eeprom_storage));
	
	memcpy(&eeprom_shadow, &eeprom, sizeof(t_eeprom_storage));
	
	if (eeprom.salt!=pgm_read_byte(&eeprom_default.eeprom.salt))
		eeprom_load_defaults();
}

void eeprom_load_defaults(void)
{
	memcpy_P(&eeprom, &eeprom_default.eeprom, sizeof(t_eeprom_storage));
	memcpy_P(&eeprom_status, &eeprom_default.eeprom_status, sizeof(t_eeprom_status));
}

void eeprom_sync_storage(void)
{
    if (memcmp(&eeprom, &eeprom_shadow, sizeof(t_eeprom_storage)))
    {
        eeprom_update_block(&eeprom, &eeprom_eemem, sizeof(t_eeprom_storage));        
        memcpy(&eeprom_shadow, &eeprom, sizeof(t_eeprom_storage));
    }
}

void eeprom_sync_status(void)
{  
    if (memcmp(&eeprom_status, &eeprom_status_shadow, sizeof(t_eeprom_status)))
    {
        eeprom_update_block(&eeprom_status, &eeprom_status_eemem, sizeof(t_eeprom_status));
	    memcpy(&eeprom_status_shadow, &eeprom_status, sizeof(t_eeprom_status));
    }
}
