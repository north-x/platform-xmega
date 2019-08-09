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

#include <stddef.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "sys/utils.h"

#if 1
// Software CRC for A4 devices 
#include <util/crc16.h>
uint16_t getID16(void)
{
	uint8_t address, temp8;
	uint16_t id = 0;
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	while (NVM.STATUS&NVM_NVMBUSY_bm);
	
	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	
	for (address=offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0);address<=offsetof(NVM_PROD_SIGNATURES_t, COORDY1);address++)
	{
		temp8 = pgm_read_byte(address);
		id = _crc_xmodem_update(id, temp8);
		__asm__ __volatile__("");
	}
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	return id;
}

#else
// Hardware CRC for A4U devices
uint16_t getID16(void)
{
	uint8_t address, temp8;
	uint16_t id = 0;
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	CRC.CTRL = CRC_RESET_RESET0_gc | CRC_SOURCE_IO_gc;
	
	while (NVM.STATUS&NVM_NVMBUSY_bm);
	
	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	
	for (address=offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0);address<=offsetof(NVM_PROD_SIGNATURES_t, COORDY1);address++)
	{
		temp8 = pgm_read_byte(address);
		CRC.DATAIN = temp8;
		id += temp8;
		__asm__ __volatile__("");
	}
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	id = (CRC.CHECKSUM1<<8) + CRC.CHECKSUM0;
	return id;
}
#endif
