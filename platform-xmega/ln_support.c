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

#include <avr/io.h>
#include <avr/wdt.h>
#include "sys/process.h"
#include "sys/etimer.h"
#include "eeprom.h"
#include "loconet/utils.h"
#include "ln_interface.h"
#include "sv.h"
#include "IdStorage.h"
#include "ln_support.h"
#include "usb_support.h"
#include "config.h"
#include "sys/utils.h"

PROCESS(ln_process, "Loconet Handler");
PROCESS(ln_ack_process, "Loconet Ack Handler");
PROCESS(ln_wdt_process, "Loconet Watchdog Handler");

static struct etimer ln_ack_timer;
static struct etimer ln_wdt_timer;
static struct etimer ln_lookup_timer;

static process_event_t ln_ack_event;

LnBuf LnBuffer;
LnBuf LnTxBuffer;

uint8_t ln_gpio_tx[LN_GPIO_BW];
uint8_t ln_gpio_tx_ack[LN_GPIO_BW];
uint8_t ln_gpio_status[LN_GPIO_BW];
uint8_t ln_gpio_status_pre[LN_GPIO_BW];
uint8_t ln_gpio_status_ack[LN_GPIO_BW];
uint8_t ln_gpio_status_flag[LN_GPIO_BW];
uint8_t ln_gpio_ack_counter[LN_GPIO_CH_COUNT];
uint8_t ln_wdt_flag;
uint8_t ln_wdt_counter __attribute__ ((section (".noinit")));
uint8_t ln_gpio_lookup[LN_LOOKUP_ENTRIES];
uint8_t ln_gpio_lookup_list[2*LN_GPIO_CH_COUNT];

extern uint16_t deviceID;

void loconet_init(void)
{
	ln_update_lookup();
	
	initLnBuf(&LnBuffer);
	initLnBuf(&LnTxBuffer);
	
	initLocoNet(&LnBuffer);
	lnTxEcho = 0;
	ACA.CTRLB = ((eeprom.ln_threshold/4)-1)&0x3F;
	
	if (eeprom.sv_serial_number==0xFFFF)
	{
		eeprom.sv_serial_number = deviceID;
		ln_update_lookup();
		eeprom_sync_storage();
		
		// Issue a software reset
		CCP = CCP_IOREG_gc;
		RST.CTRL = RST_SWRST_bm;
	}
	
	if (readSVDestinationId()==0xFFFF)
	{
		writeSVDestinationId(deviceID);
	}
	
	ln_ack_event = process_alloc_event();
	process_start(&ln_ack_process, NULL);
	
	// Start Watchdog process if requested
	if (eeprom.ln_gpio_config&(1<<LN_GPIO_CONFIG_WATCHDOG))
	{
		// Enable watchdog if the fuses are not configured
		if (!(WDT.CTRL&WDT_ENABLE_bm))
		{
			wdt_enable(WDT_PER_8KCLK_gc);
		}
		process_start(&ln_wdt_process, NULL);
	}
	else
	{
		ln_wdt_counter = 0;
	}
}

void ln_update_threshold(void)
{
	ACA.CTRLB = ((eeprom.ln_threshold/4)-1)&0x3F;
}


PROCESS_THREAD(ln_wdt_process, ev, data)
{
	static lnMsg wdt_msg;
	
	PROCESS_BEGIN();
	
	if (RST.STATUS&RST_PORF_bm)
	{
		ln_wdt_counter = 0;
		RST.STATUS = RST_PORF_bm;
	}
	
	if (RST.STATUS&RST_WDRF_bm)
	{
		if (ln_wdt_counter<0xFF)
		{
			ln_wdt_counter++;
		}
		RST.STATUS = RST_WDRF_bm;
	}
	
	wdt_msg.data[0] = 0x80;
	etimer_set(&ln_wdt_timer, deviceID/256 + 2.5*CLOCK_SECOND);
	
	while (1)
	{
		PROCESS_YIELD();

		etimer_reset(&ln_wdt_timer);

		if (!ln_wdt_flag)
		{
			uint8_t tmp = lnTxEcho;
			lnTxEcho = 1;
			sendLocoNetPacket(&wdt_msg);
			lnTxEcho = tmp;
		}
		
		ln_wdt_flag = 0;
	}
	
	PROCESS_END();
}

PROCESS_THREAD(ln_ack_process, ev, data)
{
	static uint8_t current_channel = 0;
	
	PROCESS_BEGIN();
	
	etimer_set(&ln_ack_timer, 250E-3*CLOCK_SECOND);
	
	while (1)
	{
		PROCESS_WAIT_EVENT();
		if (ev==PROCESS_EVENT_TIMER)
			etimer_reset(&ln_ack_timer);
		else if (ev==ln_ack_event)
			current_channel = (intptr_t) data;
		
		if (current_channel>=LN_GPIO_CH_COUNT)
			current_channel = 0;

		if (ln_gpio_status_ack[current_channel/8]&(1<<(current_channel%8)))
		{
			if (ln_gpio_ack_counter[current_channel]>0)
			{
				ln_gpio_tx[current_channel/8] |= (1<<(current_channel%8));
				
				if (ln_gpio_ack_counter[current_channel]<255)
				{
					ln_gpio_ack_counter[current_channel]--;
				}
			}
			else
			{
				ln_gpio_status_ack[current_channel/8] &= ~(1<<(current_channel%8));
			}
		}
		
		current_channel++;
	}
	
	PROCESS_END();
}

PROCESS_THREAD(ln_process, ev, data)
{
	lnMsg *LnPacket;
	uint8_t index;
	uint8_t source;
	
	// Mimic missing PROCESS_TIMERHANDLER(handler)
	if ((ev==PROCESS_EVENT_TIMER) /*&& (data==&ln_lookup_timer)*/)
	{
		ln_update_lookup();
	}
	
	PROCESS_BEGIN();
	
	// Initialization
	loconet_init();
	for (index=0;index<LN_GPIO_BW;index++)
	{
		ln_gpio_status[index] = eeprom_status.ln_gpio_status[index];
		ln_gpio_status_pre[index] = eeprom_status.ln_gpio_status[index];
	}
	
	if (eeprom.ln_gpio_config&(1<<LN_GPIO_CONFIG_STARTUP))
	{
		// Force transmission of current state
		for (index=0;index<LN_GPIO_BW;index++)
		{
			ln_gpio_tx[index] = 0xFF;
		}
	}
	
	while (1)
	{
		PROCESS_PAUSE();
		
		for (index=0;index<LN_GPIO_BW;index++)
		{
			eeprom_status.ln_gpio_status[index] = ln_gpio_status[index];
		}
		
		doSVDeferredProcessing();
		
		source = 0;
		LnPacket = recvLocoNetPacket();
		
		if (!LnPacket)
		{
			LnPacket = recvLnMsg(&LnTxBuffer);
			source = 1;
		}
	
		if (LnPacket)
		{   
			wdt_reset();
			ln_wdt_flag = 1;
			
			sendLocoNetPacketUSB(LnPacket);
			
			ln_gpio_process_rx(LnPacket, source);
			
			//ln_throttle_process(LnPacket);
			
			#define LN_RX_CALLBACK(fun) fun(LnPacket);
			#include "config.h"
			#undef LN_RX_CALLBACK
			
			if ((LnPacket->sz.command == OPC_GPON) && (eeprom.ln_gpio_config&(1<<LN_GPIO_CONFIG_GPON)))
			{
				// Force transmission of current state
				for (index=0;index<LN_GPIO_BW;index++)
				{
					ln_gpio_tx[index] = 0xFF;
				}
			}
			/*
			else if (BootloaderParseMessage(LnPacket)==1)
			{
				//BootloaderEnter(); // enter bootloader from running application
			}*/
			else
				processSVMessage(LnPacket);
		}
		else
		{
			ln_gpio_process_tx();
		}
	}
	
	PROCESS_END();
}

void ln_gpio_process_tx(void)
{
	uint8_t *msg;
	static uint8_t current_channel = 0;
	
	if (current_channel>=LN_GPIO_CH_COUNT)
		current_channel = 0;
		
	if (((ln_gpio_status[current_channel/8]^ln_gpio_status_pre[current_channel/8])|ln_gpio_tx[current_channel/8])&(1<<(current_channel%8)))
	{
		// Check direction
		if (eeprom.ln_gpio_dir[current_channel/8]&(1<<(current_channel%8)))
		{
			// Transmit opcode if output (dir=1)
			if (ln_gpio_status[current_channel/8]&(1<<(current_channel%8)))
			{
				msg = eeprom.ln_gpio_opcode[current_channel*2+1];
				
				if (msg[0]!=0)
				{
					if (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)
					{
						addByteLnBuf(&LnTxBuffer, msg[0]);
						addByteLnBuf(&LnTxBuffer, msg[1]);
						addByteLnBuf(&LnTxBuffer, msg[2]);
						addByteLnBuf(&LnTxBuffer, 0xFF ^ msg[0] ^ msg[1] ^ msg[2]);
						ln_gpio_status_pre[current_channel/8] |= (1<<(current_channel%8));
						ln_gpio_tx[current_channel/8] &= ~(1<<(current_channel%8));
						if ((ln_gpio_status_ack[current_channel/8]&(1<<(current_channel%8)))==0)
						{
							ln_gpio_status_ack[current_channel/8] |= (1<<(current_channel%8));
							ln_gpio_ack_counter[current_channel] = eeprom.ln_gpio_ack_count;
						}
					}
				}
				else
				{
					ln_gpio_status_pre[current_channel/8] |= (1<<(current_channel%8));
					ln_gpio_tx[current_channel/8] &= ~(1<<(current_channel%8));
				}
			}
			else
			{
				msg = eeprom.ln_gpio_opcode[current_channel*2];
				
				if (msg[0]!=0)
				{
					if (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)
					{
						addByteLnBuf(&LnTxBuffer, msg[0]);
						addByteLnBuf(&LnTxBuffer, msg[1]);
						addByteLnBuf(&LnTxBuffer, msg[2]);
						addByteLnBuf(&LnTxBuffer, 0xFF ^ msg[0] ^ msg[1] ^ msg[2]);
						ln_gpio_status_pre[current_channel/8] &= ~(1<<(current_channel%8));
						ln_gpio_tx[current_channel/8] &= ~(1<<(current_channel%8));
						if ((ln_gpio_status_ack[current_channel/8]&(1<<(current_channel%8)))==0)
						{
							ln_gpio_status_ack[current_channel/8] |= (1<<(current_channel%8));
							ln_gpio_ack_counter[current_channel] = eeprom.ln_gpio_ack_count;
						}
					}
				}
				else
				{
					ln_gpio_status_pre[current_channel/8] &= ~(1<<(current_channel%8));
					ln_gpio_tx[current_channel/8] &= ~(1<<(current_channel%8));
				}
			}
		}
		else
		{
			// Transmit status if input (dir=0)
			if (ln_gpio_status[current_channel/8]&(1<<(current_channel%8)))
			{
				if (ln_create_message(eeprom.ln_gpio_opcode[current_channel*2+1]))
				{
					ln_gpio_status_pre[current_channel/8] |= (1<<(current_channel%8));
					ln_gpio_tx[current_channel/8] &= ~(1<<(current_channel%8));
					if ((ln_gpio_status_ack[current_channel/8]&(1<<(current_channel%8)))==0)
					{
						ln_gpio_status_ack[current_channel/8] |= (1<<(current_channel%8));
						ln_gpio_ack_counter[current_channel] = eeprom.ln_gpio_ack_count;
					}
				}
			}
			else
			{
				if (ln_create_message(eeprom.ln_gpio_opcode[current_channel*2]))
				{
					ln_gpio_status_pre[current_channel/8] &= ~(1<<(current_channel%8));
					ln_gpio_tx[current_channel/8] &= ~(1<<(current_channel%8));
					if ((ln_gpio_status_ack[current_channel/8]&(1<<(current_channel%8)))==0)
					{
						ln_gpio_status_ack[current_channel/8] |= (1<<(current_channel%8));
						ln_gpio_ack_counter[current_channel] = eeprom.ln_gpio_ack_count;
					}
				}
			}
		}
	}
	else if (ln_gpio_tx_ack[current_channel/8]&(1<<(current_channel%8)))
	{
		if (ln_gpio_status[current_channel/8]&(1<<(current_channel%8)))
		{
			if (ln_create_message_ack(eeprom.ln_gpio_opcode[current_channel*2+1]))
			{
				ln_gpio_tx_ack[current_channel/8] &= ~(1<<(current_channel%8));
			}
		}
		else
		{
			if (ln_create_message_ack(eeprom.ln_gpio_opcode[current_channel*2]))
			{
				ln_gpio_tx_ack[current_channel/8] &= ~(1<<(current_channel%8));
			}
		}
	}
	
	current_channel++;
}

uint8_t ln_create_message(uint8_t *msg)
{
	LN_STATUS status = LN_UNKNOWN_ERROR;
	uint8_t msg_tx[4];
	
	if (msg[0]==0)
		return 1;
		
	switch (msg[0])
	{
		// translate switch request to switch report
		case OPC_SW_REQ:
			//msg0 = OPC_SW_REP;
			//msg2 = (msg[2]&0xF)|OPC_SW_REP_SW|OPC_SW_REP_INPUTS|((msg[2]&OPC_SW_REQ_DIR)?OPC_SW_REP_HI:0);
			msg_tx[0] = OPC_SW_REP;
			msg_tx[1] = msg[1];
			msg_tx[2] = (msg[2]&0xF)|OPC_SW_REP_SW|OPC_SW_REP_INPUTS|((msg[2]&OPC_SW_REQ_DIR)?OPC_SW_REP_HI:0);
			status = sendLocoNet4BytePacket(msg_tx[0], msg_tx[1], msg_tx[2]);
			if (status==LN_DONE)
			{
				msg_tx[3] = 0xFF ^ msg_tx[0] ^ msg_tx[1] ^ msg_tx[2];
				addMsgLnBuf(&LnTxBuffer, (lnMsg *)msg_tx);
			}
			break;
		default:
			status = sendLocoNet4BytePacket(msg[0], msg[1], msg[2]);
			if (status==LN_DONE)
			{
				msg_tx[0] = msg[0];
				msg_tx[1] = msg[1];
				msg_tx[2] = msg[2];
				msg_tx[3] = 0xFF ^ msg_tx[0] ^ msg_tx[1] ^ msg_tx[2];
				addMsgLnBuf(&LnTxBuffer, (lnMsg *)msg);
			}
	}
	
	return (status==LN_DONE)?1:0;
}

uint8_t ln_create_message_ack(uint8_t *msg)
{
	LN_STATUS status = LN_UNKNOWN_ERROR;
	uint8_t msg_tx[4];
	
	if (msg[0]==0)
	{
		return 1;
	}
	
	if (msg[0]==OPC_SW_ACK)
	{
		msg_tx[0] = OPC_LONG_ACK;
		msg_tx[1] = msg[0]&0x7F;
		msg_tx[2] = 0x7F;
		
		status = sendLocoNet4BytePacket(msg_tx[0], msg_tx[1], msg_tx[2]);
		if (status==LN_DONE)
		{
			msg_tx[3] = 0xFF ^ msg_tx[0] ^ msg_tx[1] ^ msg_tx[2];
			addMsgLnBuf(&LnTxBuffer, (lnMsg *)msg_tx);
		}
		
		return (status==LN_DONE)?1:0;
	}
	
	if (eeprom.ln_gpio_ack_count>0)
	{
		msg_tx[0] = msg[0];
		msg_tx[1] = msg[1];
		msg_tx[2] = msg[2]^(1<<6);
		
		status = sendLocoNet4BytePacket(msg_tx[0], msg_tx[1], msg_tx[2]);
		if (status==LN_DONE)
		{
			msg_tx[3] = 0xFF ^ msg_tx[0] ^ msg_tx[1] ^ msg_tx[2];
			addMsgLnBuf(&LnTxBuffer, (lnMsg *)msg_tx);
		}
		
		return (status==LN_DONE)?1:0;
	}
	else
	{
		return 1;
	}
}

uint8_t ln_is_ack_message(uint8_t *msg)
{
	switch (msg[0])
	{		
		case OPC_SW_REQ:
			return (msg[2]&(1<<6))!=0;
		case OPC_SW_REP:
		case OPC_INPUT_REP:
			return (msg[2]&(1<<6))==0;
		default:
			return 1==0;
	}
}

void ln_gpio_process_rx(lnMsg *LnPacket, uint8_t source)
{
	uint8_t index;
	uint8_t ack, cmd;
	
	if (!LnPacket)
		return;
	
	if (getLnMsgSize(LnPacket)>4)
		return;
	
	uint8_t checksum = getLnMsgChecksum(LnPacket) & LN_LOOKUP_MASK;
	
	// Loop over all valid entries
	for (index = ln_gpio_lookup[checksum];index!=0xFF;index = ln_gpio_lookup_list[index])
	{
		// 1. Find a matching opcode
		// - either a direct match, or
		// - by matching a switch report/request pair
		if ((LnPacket->sr.command==eeprom.ln_gpio_opcode[index][0])
		|| ((LnPacket->sr.command==OPC_SW_REP) && (eeprom.ln_gpio_opcode[index][0]==OPC_SW_REQ)))
		{
			
		}
		else
			continue;
		
		// 2. Check first address byte
		if (LnPacket->srq.sw1!=eeprom.ln_gpio_opcode[index][1])
			continue;
			
		ack = 0;
		cmd = 0;
		
		// 3. Handle the opcode
		switch (LnPacket->sr.command)
		{
			case OPC_SW_REQ:
				if ((LnPacket->srq.sw2&(~(1<<6)))!=(eeprom.ln_gpio_opcode[index][2]&(~(1<<6))))
					continue;
				
				if ((LnPacket->srq.sw2&(1<<6))==0)
				{
					// Command received, send ack
					cmd = 1;
					ack = 1;
				}
				else
				{
					// Ack received and direction output (==1)
					ack = 1;
				}
				break;
			case OPC_SW_REP:
				if (eeprom.ln_gpio_opcode[index][0]==OPC_SW_REQ)
				{
					if ((LnPacket->srq.sw2&0xF)!=(eeprom.ln_gpio_opcode[index][2]&0xF))
					{
						continue;
					}
				
					// We received the ACK, so we can clear the flag and stop transmitting
					if ((LnPacket->srq.sw2&(1<<6))==0)
					{
						ack = 1;
					}
				}
				else
				{
					if ((LnPacket->srq.sw2&(~(1<<6)))!=(eeprom.ln_gpio_opcode[index][2]&(~(1<<6))))
						continue;
					
					if ((LnPacket->srq.sw2&(1<<6))!=0)
					{
						cmd = 1;
						ack = 1;
					}
					else
					{
						// Ack
						ack = 1;
					}
				}	
				break;
			case OPC_INPUT_REP:
				if ((LnPacket->srq.sw2&(~(1<<6)))!=(eeprom.ln_gpio_opcode[index][2]&(~(1<<6))))
					continue;
					
				if ((LnPacket->srq.sw2&(1<<6))!=0)
				{
					cmd = 1;
					ack = 1;
				}
				else
				{
					// Ack received and direction output (==1)
					// Ack
					ack = 1;
				}
				break;
			default:
				if (LnPacket->srq.sw2!=eeprom.ln_gpio_opcode[index][2])
					continue;
				cmd = 1;
				break;
		}
		
		// No ACK handling if source is internal
		if (source==1)
			ack = 0;
		
		// We have a match, now set or clear the status
		if (cmd)
		{
			if ((index%2)==0)
			{
				ln_gpio_status[index/16] &= ~(1<<((index/2)%8));
				ln_gpio_status_pre[index/16] &= ~(1<<((index/2)%8));
			}
			else
			{
				ln_gpio_status[index/16] |= (1<<((index/2)%8));
				ln_gpio_status_pre[index/16] |= (1<<((index/2)%8));
			}
			
			ln_gpio_status_flag[index/16] |= (1<<((index/2)%8));
		
			if (ack && (eeprom.ln_gpio_ack_count>0))
			{
				ln_gpio_tx_ack[index/16] |= (1<<((index/2)%8));
				process_post(&ln_ack_process, ln_ack_event, (void *) (index/2));
			}
		}
		else
		{
			if (ack)
			{
				ln_gpio_status_ack[index/16] &= ~(1<<((index/2)%8));
			}
		}
	}
}

void ln_create_opcode(uint8_t *buf, uint8_t opc, uint16_t addr)
{
	if (addr==0)
	{
		buf[0] = 0;
		buf[3] = 0;
		
		return;
	}
	
	switch (opc)
	{
		case OPC_SW_REQ:
			addr--;
			buf[0] = OPC_SW_REQ;
			buf[3] = OPC_SW_REQ;
			buf[1] = addr&0x7F;
			buf[4] = buf[1];
			buf[5] = (addr>>7)&0x7F;
			buf[2] = buf[5]|OPC_SW_REQ_DIR;
			break;
		case OPC_INPUT_REP:
			addr--;
			buf[0] = OPC_INPUT_REP;
			buf[3] = OPC_INPUT_REP;
			buf[1] = (addr>>1)&0x7F;
			buf[4] = buf[1];
			buf[2] = ((addr>>8)&0x7F)|OPC_INPUT_REP_CB|((addr&1)?0x20:0);
			buf[5] = buf[2]|OPC_INPUT_REP_HI;
			break;
	}
}
uint8_t getLnMsgChecksum(lnMsg *msg)
{
	uint8_t index;
	
	uint8_t chk = 0xFF;
	uint8_t len = getLnMsgSize(msg) - 1;
	
	for (index=0;index<len;index++)
	{
		chk ^= msg->data[index];
	}
	
	return chk;
}

void ln_trigger_lookup_update(void)
{
	etimer_set(&ln_lookup_timer, 2*CLOCK_SECOND);
}

void ln_update_lookup(void)
{
	uint8_t index, checksum;
	for (index=0;index<sizeof(ln_gpio_lookup);index++)
	{
		ln_gpio_lookup[index] = 0xFF;
	}
	
	for (index=0;index<(2*LN_GPIO_CH_COUNT);index++)
	{
		lnMsg * msg = (lnMsg *)eeprom.ln_gpio_opcode[index];
		
		if (msg->data[0]<128)
			continue;

		checksum = getLnMsgChecksum(msg) & LN_LOOKUP_MASK;
				
		if (ln_gpio_lookup[checksum]==0xFF)
		{
			ln_gpio_lookup[checksum] = index;
			ln_gpio_lookup_list[index] = 0xFF;
		}
		else
		{
			uint8_t idx = ln_gpio_lookup[checksum];
			
			lookup_loop:
			
			if (ln_gpio_lookup_list[idx]==0xFF)
			{
				ln_gpio_lookup_list[idx] = index;
			}
			else
			{
				idx = ln_gpio_lookup_list[idx];
				goto lookup_loop;
			}
			
			ln_gpio_lookup_list[index] = 0xFF;
		}
	}
}

void lns_sv_cmd_callback(uint8_t cmd)
{
	uint8_t index;
	
	if (cmd==255)
	{
		// Force transmission of current state
		for (index=0;index<LN_GPIO_BW;index++)
		{
			ln_gpio_tx[index] = 0xFF;
		}
	}
}