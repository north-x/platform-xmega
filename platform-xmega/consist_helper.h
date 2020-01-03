/*
 * Copyright (c) 2020, Manuel Vetterli
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

/************************************************************************/
/* Module Configuration                                                 */
/************************************************************************/
#ifdef CONFIGURATION

/*
 *	Autostart List
 *
 *	List of all processes that are started automatically on startup
 *
 */
#ifdef AUTOSTART_CFG
&consist_helper_process,
#endif

/*
 *	SV Configuration Table
 *
 *	List of all configuration variables defined by this module
 *
 */
#ifdef SV_CFG
SV(285, "Address 1", loco_addr[0], 0)
SV(286, "Address 2", loco_addr[1], 0)
SV(287, "Slot 1", loco_slot[0], 0)
SV(288, "Slot 2", loco_slot[1], 0)
SV(289, "Loco Cmd", loco_index, 0)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
//uint8_t servo_multipos_opcode;
//uint8_t servo_multipos[2][2];
#endif

/*
 *	EEPROM Status Variable Definition
 */
#ifdef EEPROM_STATUS_CFG
//uint8_t relay_request;
#endif

/*
 *	EEPROM Configuration Variable Default Configuration
 */
#ifdef EEPROM_DEFAULT
//.servo_multipos_opcode = 0,
//.servo_multipos = {{64, 192}, {64, 192}},
#endif

/*
 *	EEPROM Status Variable Default Configuration
 */
#ifdef EEPROM_STATUS_DEFAULT
//.relay_request = 0,
#endif

/*
 * LN Receive Callback Definition
 *
 * Function to be called when a valid packet was received
 */
#ifdef LN_RX_CALLBACK
LN_RX_CALLBACK(ln_consist_helper_callback)
#endif

/*
 * SV CMD Callback Definition
 *
 * Function to be called when SV#5 is written
 */
#ifdef SV_CMD_CALLBACK
//SV_CMD_CALLBACK(ln_sv_cmd_callback)
#endif


#else
/************************************************************************/
/* Module Header File                                                   */
/************************************************************************/
#ifndef consist_helper_H_
#define consist_helper_H_


PROCESS_NAME(consist_helper_process);

void ln_consist_helper_callback(lnMsg *LnPacket);
void ln_sv_cmd_callback(uint8_t cmd);

extern uint16_t loco_addr[];
extern uint8_t loco_slot[];
extern uint8_t loco_index;
extern uint8_t loco_cmd;

typedef struct t_slot_info 
{
	uint8_t stat;
	uint8_t speed;
	uint16_t address;
} t_slot_info;

#define PU_SLOTS_CAN_LINK 8
#define PU_SLOTS_CAN_UNLINK 9

#endif /* consist_helper_H_ */
#endif