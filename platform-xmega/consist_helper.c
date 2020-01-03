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

#include <avr/io.h>
#include "sys/process.h"
#include "sys/etimer.h"
#include "ln_interface.h"
#include "consist_helper.h"
#include "config.h"
#include "eeprom.h"

PROCESS(consist_helper_process,"Consist Helper Application Process");

extern LnBuf LnBuffer2;
extern LnBuf LnBuffer3;

uint16_t loco_addr[2];
uint8_t loco_slot[2];
uint8_t loco_index;
uint8_t loco_cmd;

t_slot_info slot_info[120];

clock_time_t last_activity[2];

PROCESS_THREAD(consist_helper_process, ev, data)
{
	static uint8_t slots_can_link;
	static uint8_t slots_can_unlink;
	static uint8_t slot_index;
	PROCESS_BEGIN();
	
	// Initialization
	last_activity[0] = clock_time();
	last_activity[1] = last_activity[0];
	
	while (1)
	{
		PROCESS_PAUSE();
		
		clock_time_t now = clock_time();
		
		if (now-last_activity[0]>30*CLOCK_SECOND)
		{
			last_activity[0] = now;
			loco_slot[0] = 0;
			loco_addr[0] = 0;
		}

		if (now-last_activity[1]>30*CLOCK_SECOND)
		{
			last_activity[1] = now;
			loco_slot[1] = 0;
			loco_addr[1] = 0;
		}
		
		slots_can_link = 0;
		slots_can_unlink = 0;
		port_user &= ~((1<<PU_SLOTS_CAN_LINK)|(1<<PU_SLOTS_CAN_UNLINK));
		
		slot_index++;
		
		if (slot_index>=0x70)
			slot_index = 1;
		
		// Case 1: Fred w/ CONSIST_TOP plugged in --> find corresponding CONSIST_SUB
		if ((loco_slot[0]!=0) &&
			(loco_slot[1]==0) &&
			((slot_info[loco_slot[0]].stat&CONSIST_MASK)==CONSIST_TOP)
			)
		{
			// Find corresponding CONSIST_SUB
			if (((slot_info[slot_index].stat&CONSIST_MASK)==CONSIST_SUB) &&
				(slot_info[slot_index].speed==loco_slot[0])
				)
			{
				loco_slot[1] = slot_index;
				loco_addr[1] = slot_info[slot_index].address;
			}
		}
		
		// Case 2: Fred w/ CONSIST_SUB plugged in --> assign CONSIST_TOP
		if ((loco_slot[0]!=0) &&
			(loco_slot[1]==0) &&
			((slot_info[loco_slot[0]].stat&CONSIST_MASK)==CONSIST_SUB)
			)
		{
			// Check corresponding CONSIST_TOP
			if ((slot_info[slot_info[loco_slot[0]].speed].stat&CONSIST_MASK)==CONSIST_TOP)
			{
				// Load and swap entries such that 0->TOP, 1->SUB
				slot_index = loco_slot[0];
				loco_slot[0] = slot_info[slot_index].speed;
				loco_addr[0] = slot_info[loco_slot[0]].address;
				loco_slot[1] = slot_index;
				loco_addr[1] = slot_info[slot_index].address;
			}
		}
		
		// Case 3: Both Freds plugged in w/o consist
		if ((loco_slot[0]!=0) &&
			(loco_slot[1]!=0) &&
			((slot_info[loco_slot[0]].stat&CONSIST_MASK)==CONSIST_NO) &&
			((slot_info[loco_slot[1]].stat&CONSIST_MASK)==CONSIST_NO)
			)
		{
			slots_can_link = 1;
			port_user |= (1<<PU_SLOTS_CAN_LINK);
		}
		
		// Case 4: Both Freds plugged in w/ consist
		if ((loco_slot[0]!=0) &&
			(loco_slot[1]!=0) &&
			((slot_info[loco_slot[0]].stat&CONSIST_MASK)==CONSIST_TOP) &&
			((slot_info[loco_slot[1]].stat&CONSIST_MASK)==CONSIST_SUB)
			)
		{
			slots_can_unlink = 1;
			port_user |= (1<<PU_SLOTS_CAN_UNLINK);
		}
		
		// Case 5: Both Freds plugged in w/ consist, but in wrong order
		if ((loco_slot[0]!=0) &&
			(loco_slot[1]!=0) &&
			((slot_info[loco_slot[1]].stat&CONSIST_MASK)==CONSIST_TOP) &&
			((slot_info[loco_slot[0]].stat&CONSIST_MASK)==CONSIST_SUB)
			)
		{
			slot_index = loco_slot[0];
			loco_slot[0] = loco_slot[1];
			loco_addr[0] = slot_info[loco_slot[0]].address;
			loco_slot[1] = slot_index;
			loco_addr[1] = slot_info[loco_slot[1]].address;
		}
		
		if ((slots_can_link!=0) && ((port_di&(1<<7))!=0))
		{
			sendLocoNet4BytePacket(OPC_LINK_SLOTS, loco_slot[1], loco_slot[0]);
			loco_slot[0] = 0;
			loco_slot[1] = 0;
			loco_addr[0] = 0;
			loco_addr[1] = 0;
		}
		
		if ((slots_can_unlink!=0) && ((port_di&(1<<7))!=0))
		{
			sendLocoNet4BytePacket(OPC_UNLINK_SLOTS, loco_slot[1], loco_slot[0]);
			loco_slot[0] = 0;
			loco_slot[1] = 0;
			loco_addr[0] = 0;
			loco_addr[1] = 0;
		}
		
		loco_index = slots_can_link + slots_can_unlink + slots_can_unlink;

	}
	
	PROCESS_END();
}

void ln_consist_helper_callback(lnMsg *LnPacket)
{
	lnMsg *msg = recvLnMsg(&LnBuffer2);
	
	if (msg!=NULL)
	{
		last_activity[0] = clock_time();
	}
	
	if (msg->data[0]==OPC_LOCO_ADR)
	{
		loco_addr[0] = (msg->la.adr_hi<<7) + msg->la.adr_lo;
		loco_slot[0] = 0;
	}
	else if ((msg->sm.command==OPC_MOVE_SLOTS) &&
			(msg->sm.src==0) &&
			(msg->sm.dest==0))
	{
		loco_addr[0] = 0xFFFF;
		loco_slot[0] = 0;
	}
	msg = recvLnMsg(&LnBuffer3);
	
	if (msg!=NULL)
	{
		last_activity[1] = clock_time();
	}
	
	if (msg->data[0]==OPC_LOCO_ADR)
	{
		loco_addr[1] = (msg->la.adr_hi<<7) + msg->la.adr_lo;
		loco_slot[1] = 0;
	}
	else if ((msg->sm.command==OPC_MOVE_SLOTS) &&
			(msg->sm.src==0) &&
			(msg->sm.dest==0))
	{
		loco_addr[1] = 0xFFFF;
		loco_slot[1] = 0;
	}
	
	if (LnPacket->data[0]==OPC_SL_RD_DATA)
	{
		uint16_t addr = (LnPacket->sd.adr2<<7) + LnPacket->sd.adr;
		
		slot_info[LnPacket->sd.slot].address = addr;
		slot_info[LnPacket->sd.slot].stat = LnPacket->sd.stat;
		slot_info[LnPacket->sd.slot].speed = LnPacket->sd.spd;
		
		if ((loco_addr[0]==addr) || (loco_addr[0]==0xFFFF))
		{
			loco_addr[0] = addr;
			loco_slot[0] = LnPacket->sd.slot;
		}
		else if ((loco_addr[1]==addr) || (loco_addr[1]==0xFFFF))
		{
			loco_addr[1] = addr;
			loco_slot[1] = LnPacket->sd.slot;
		}
	}
	
	loco_cmd = 0;
}
