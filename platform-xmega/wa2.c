/*
 * Copyright (c) 2017, Manuel Vetterli
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
#include "ln_interface.h"
#include "wa2.h"
#include "config.h"
#include "eeprom.h"

PROCESS(wa2_process,"WA2 Application Process");
PROCESS(relay_process,"Relay Control Process");
PROCESS(signal_process,"Signal Control Process");

rwSlotDataMsg rSlot;
uint8_t relay_state;
uint8_t relay_request;
uint8_t relay_cmd;
uint16_t servo_timeout_shadow;
uint8_t servo_config_mode_active;
uint8_t servo_config_state;
clock_time_t servo_config_mode_timeout;
clock_time_t wa2_config_mode_timer;

static struct etimer relay_timer;

PROCESS_THREAD(wa2_process, ev, data)
{
	
	PROCESS_BEGIN();
	
	// Initialization
	rSlot.slot = 0xFF;
	servo_timeout_shadow = eeprom.data.servo_timeout;
	
	while (1)
	{
		PROCESS_PAUSE();
		
		// Servo 1
		// Check if a new command was received
		if (ln_gpio_status_flag[0]&(1<<0))
		{
			// Set new position accordingly
			if (ln_gpio_status[0]&(1<<0))
			{
				servo[0].position_setpoint = 255;
			}
			else
			{
				servo[0].position_setpoint = 0;
			}

			// Turn on relay if position greater than 127
			if (servo[0].position_actual>127)
			{
				relay_request |= (1<<0);
			}
			else
			{
				relay_request &= ~(1<<0);		
			}
			
			// If we reached the final position, transmit a switch report
			if (servo[0].position_actual==servo[0].position_setpoint)
			{
				ln_gpio_tx[0] |= (1<<0);
				ln_gpio_status_flag[0] &= ~(1<<0);
			}
			
		}
		else if ((eeprom.data.servo_multipos_opcode&0x0F) && (ln_gpio_status_flag[(eeprom.data.servo_multipos_opcode&0xF)/8]&(1<<((eeprom.data.servo_multipos_opcode&0xF)%8))))
		{
			// Set new position accordingly
			if (ln_gpio_status[(eeprom.data.servo_multipos_opcode&0xF)/8]&(1<<((eeprom.data.servo_multipos_opcode&0xF)%8)))
			{
				servo[0].position_setpoint = eeprom.data.servo_multipos[0][1];
			}
			else
			{
				servo[0].position_setpoint = eeprom.data.servo_multipos[0][0];
			}
			// Turn on relay if position greater than 127
			if (servo[0].position_actual>127)
			{
				relay_request |= (1<<0);
			}
			else
			{
				relay_request &= ~(1<<0);
			}
			
			// If we reached the final position, transmit a switch report
			if (servo[0].position_actual==servo[0].position_setpoint)
			{
				ln_gpio_tx[(eeprom.data.servo_multipos_opcode&0xF)/8] |= (1<<((eeprom.data.servo_multipos_opcode&0xF)%8));
				ln_gpio_status_flag[(eeprom.data.servo_multipos_opcode&0xF)/8] &= ~(1<<((eeprom.data.servo_multipos_opcode&0xF)%8));
			}
			
		}
		
		// Servo 2
		// Check if a new command was received
		if (ln_gpio_status_flag[0]&(1<<1))
		{
			// Set new position accordingly
			if (ln_gpio_status[0]&(1<<1))
			{
				servo[1].position_setpoint = 255;
			}
			else
			{
				servo[1].position_setpoint = 0;
			}

			// Turn on relay if position greater than 127
			if (servo[1].position_actual>127)
			{
				port_user |= (1<<PU_RELAY2_REQ);
			}
			else
			{
				port_user &= ~(1<<PU_RELAY2_REQ);
			}
			
			// If we reached the final position, transmit the new status
			if (servo[1].position_actual==servo[1].position_setpoint)
			{
				ln_gpio_tx[0] |= (1<<1);
				ln_gpio_status_flag[0] &= ~(1<<1);
			}
			
		}
		else if ((eeprom.data.servo_multipos_opcode&0xF0) && (ln_gpio_status_flag[(eeprom.data.servo_multipos_opcode>>4)/8]&(1<<((eeprom.data.servo_multipos_opcode>>4)%8))))
		{
			// Set new position accordingly
			if (ln_gpio_status[(eeprom.data.servo_multipos_opcode>>4)/8]&(1<<((eeprom.data.servo_multipos_opcode>>4)%8)))
			{
				servo[1].position_setpoint = eeprom.data.servo_multipos[1][1];
			}
			else
			{
				servo[1].position_setpoint = eeprom.data.servo_multipos[1][0];
			}
			// Turn on relay if position greater than 127
			if (servo[1].position_actual>127)
			{
				port_user |= (1<<PU_RELAY2_REQ);
			}
			else
			{
				port_user &= ~(1<<PU_RELAY2_REQ);
			}
			
			// If we reached the final position, transmit a switch report
			if (servo[1].position_actual==servo[1].position_setpoint)
			{
				ln_gpio_tx[(eeprom.data.servo_multipos_opcode>>4)/8] |= (1<<((eeprom.data.servo_multipos_opcode>>4)%8));
				ln_gpio_status_flag[(eeprom.data.servo_multipos_opcode>>4)/8] &= ~(1<<((eeprom.data.servo_multipos_opcode>>4)%8));
			}
			
		}
		
		clock_time_t now = clock_time();
		
		if (ACA.STATUS&AC_AC0STATE_bm)
		{
			wa2_config_mode_timer = now;
		}
		else
		{
			if (now - wa2_config_mode_timer > (int32_t) 4*CLOCK_SECOND)
			{
				if (rSlot.slot!=0)
				{
					wdt_reset();
				}
				rSlot.slot = 0;
				wa2_config_mode_timer = now;
			}
		}
		
		if (servo_config_mode_active==2)
		{
			if (now - servo_config_mode_timeout > (int32_t) 60*CLOCK_SECOND)
			{
				eeprom.data.servo_timeout = servo_timeout_shadow;
				servo_config_mode_active = 0;
			}
		}
	}
	
	PROCESS_END();
}

void ln_throttle_process(lnMsg *LnPacket)
{
	static lnMsg SendPacket;
	static uint16_t servo_base;
	static uint16_t *servo_act;
	static int8_t servo_mult;
	static uint8_t *servo_speed;
	static uint8_t servo_index;
	static uint8_t dirf_changes_latched;
	
	switch (LnPacket->data[0])
	{
		case OPC_LOCO_ADR:
			rSlot.adr = eeprom.data.sv_destination_id&0x7F;
			rSlot.adr2 = (eeprom.data.sv_destination_id>>7)&0x7F;
			rSlot.stat = LOCO_IDLE | DEC_MODE_128;
			rSlot.id1 = 0;
			rSlot.id2 = 0;

			if ((LnPacket->la.adr_hi==rSlot.adr2) && (LnPacket->la.adr_lo==rSlot.adr))
			{
				rSlot.slot = (eeprom.data.sv_destination_id&0x7F)?(eeprom.data.sv_destination_id&0x7F):1;
				SendPacket.sd.command   = OPC_SL_RD_DATA  ; //opcode
				SendPacket.sd.mesg_size = 14              ; // length
				SendPacket.sd.slot      = rSlot.slot      ; // slot    2
				SendPacket.sd.stat      = rSlot.stat      ; // stat    3
				SendPacket.sd.adr       = rSlot.adr       ; // adr     4
				SendPacket.sd.spd       = rSlot.spd       ; // spd     5
				SendPacket.sd.dirf      = rSlot.dirf      ; // dirf    6
				SendPacket.sd.trk       = rSlot.trk       ; // trk     7
				SendPacket.sd.ss2       = rSlot.ss2       ; // ss2     8
				SendPacket.sd.adr2      = rSlot.adr2      ; // adr2    9
				SendPacket.sd.snd       = rSlot.snd       ; // snd    10
				SendPacket.sd.id1       = rSlot.id1       ; // id1    11
				SendPacket.sd.id2       = rSlot.id2       ; // id2    12
					
				sendLocoNetPacket(&SendPacket);
			}
			else
			{
				rSlot.slot = 0xFF;
			}
			break;
		case OPC_MOVE_SLOTS:
			if (LnPacket->sm.src==LnPacket->sm.dest && LnPacket->sm.src==rSlot.slot)
			{
				// Reset state machine
				servo_config_state = 0;
				dirf_changes_latched = 0;
				
				if (rSlot.slot==0)
				{
					rSlot.slot = (eeprom.data.sv_destination_id&0x7F)?(eeprom.data.sv_destination_id&0x7F):1;
					rSlot.adr = eeprom.data.sv_destination_id&0x7F;
					rSlot.adr2 = (eeprom.data.sv_destination_id>>7)&0x7F;
				}
				
				servo_index = 0;
				servo_act = &servo[0].min;
				servo_base = *servo_act;
				servo_mult = 1;
				servo_speed = &servo[0].time_ratio;
				rSlot.stat = DEC_MODE_128 | LOCO_IN_USE;
				SendPacket.sd.command   = OPC_SL_RD_DATA  ; //opcode
				SendPacket.sd.mesg_size = 14              ; // length
				SendPacket.sd.slot      = rSlot.slot      ; // slot    2
				SendPacket.sd.stat      = rSlot.stat      ; // stat    3
				SendPacket.sd.adr       = rSlot.adr       ; // adr     4
				SendPacket.sd.spd       = rSlot.spd       ; // spd     5
				SendPacket.sd.dirf      = rSlot.dirf      ; // dirf    6
				SendPacket.sd.trk       = rSlot.trk       ; // trk     7
				SendPacket.sd.ss2       = rSlot.ss2       ; // ss2     8
				SendPacket.sd.adr2      = rSlot.adr2      ; // adr2    9
				SendPacket.sd.snd       = rSlot.snd       ; // snd    10
				SendPacket.sd.id1       = rSlot.id1       ; // id1    11
				SendPacket.sd.id2       = rSlot.id2       ; // id2    12
				
				if (LnPacket->sm.src==0)
				{
					if (sendLocoNetPacket(&SendPacket)!=LN_DONE)  // , LN_BACKOFF_INITIAL + (eeprom.sv_destination_id % (uint8_t) 10))
						rSlot.slot = 0;
				}
				else
					sendLocoNetPacket(&SendPacket);
			}
			break;
		case OPC_WR_SL_DATA:
			rSlot.id1 = LnPacket->sd.id1;
			rSlot.id2 = LnPacket->sd.id2;
			sendLocoNet4BytePacket(OPC_LONG_ACK, OPC_WR_SL_DATA&0x7F, 0);
			break;
		case OPC_LOCO_DIRF:
			if (LnPacket->ldf.slot==rSlot.slot)
			{
				servo_config_mode_timeout = clock_time();
				uint8_t dirf_changes = rSlot.dirf^LnPacket->ldf.dirf;
				dirf_changes_latched |= dirf_changes;
				
				switch (servo_config_state)
				{
					// State 0: wait for user to press F1 for servo 1 or F2 for servo 2.
					// If F0 has been pressed previously, then additionally load the default settings of the respective servo.
					case 0:
						// Break, if neither F1, nor F2 was pressed
						if ((dirf_changes&((1<<0)|(1<<1)))==0)
						{
							break;
						}
						
						servo_config_state = 1;
						if (servo_config_mode_active==0)
						{
							servo_timeout_shadow = eeprom.data.servo_timeout;
							eeprom.data.servo_timeout = 0;
						}

						servo_config_mode_active = 2;
						servo_config_mode_timeout = clock_time();
						
						// F1: Select Servo 1
						if (dirf_changes&(1<<0))
						{
							servo_index = 0;
						}
						// F2: Select Servo 2
						else if (dirf_changes&(1<<1))
						{
							servo_index = 1;
						}
					
						// F0: Load EEPROM Settings
						if (dirf_changes_latched&(1<<4))
						{
							servo[servo_index].min = eeprom.data.servo_min[servo_index];
							servo[servo_index].max = eeprom.data.servo_max[servo_index];
							servo[servo_index].time_ratio = eeprom.data.servo_time_ratio[servo_index];
						}
						
						// Direction 0/forward = right 1/reversed=left
						if ((LnPacket->ldf.dirf&(1<<5))==0)
						{
							ln_gpio_status[0] |= (1<<servo_index);
							ln_gpio_status_pre[0] |= (1<<servo_index);
							ln_gpio_status_flag[0] |= (1<<servo_index);
							servo_act = &servo[servo_index].max;
							servo_speed = &servo[servo_index].time_ratio;
						}
						else
						{
							ln_gpio_status[0] &= ~(1<<servo_index);
							ln_gpio_status_pre[0] &= ~(1<<servo_index);
							ln_gpio_status_flag[0] |= (1<<servo_index);
							servo_act = &servo[servo_index].min;
							servo_speed = &servo[servo_index].time_ratio;
						}
						
						servo_base = *servo_act;
						servo_mult = 1;
						
						break;
					case 1:
							
						// Changes in DIR
						if (dirf_changes&(1<<5))
						{
							// Restore "unsaved" position
							*servo_act = servo_base;
								
							// Reset multiplier
							servo_mult = 1;
							
							// Direction 0/forward = right 1/reversed=left
							if ((LnPacket->ldf.dirf&(1<<5))==0)
							{
								ln_gpio_status[0] |= (1<<servo_index);
								ln_gpio_status_pre[0] |= (1<<servo_index);
								ln_gpio_status_flag[0] |= (1<<servo_index);
								servo_act = &servo[servo_index].max;
								servo_speed = &servo[servo_index].time_ratio;
							}
							else
							{
								ln_gpio_status[0] &= ~(1<<servo_index);
								ln_gpio_status_pre[0] &= ~(1<<servo_index);
								ln_gpio_status_flag[0] |= (1<<servo_index);
								servo_act = &servo[servo_index].min;
								servo_speed = &servo[servo_index].time_ratio;
							}
							
							servo_base = *servo_act;
						}
							
						// F1
						if ((dirf_changes&(1<<0)))
						{
							switch (servo_mult)
							{
								case 1:
									servo_mult = 2;
									break;
								case 2:
									servo_mult = 3;
									break;
								case 3:
								default:
									servo_mult = 1;
									break;
							}
						}
							
						// F2
						if ((dirf_changes&(1<<1)))
						{
							switch (servo_mult)
							{
								case -1:
									servo_mult = -2;
									break;
								case -2:
									servo_mult = -3;
									break;
								case -3:
								default:
									servo_mult = -1;
									break;
							}
						}
						
						// F3 Increase Speed
						if ((dirf_changes&(1<<2)))
						{
							if (*servo_speed>1)
								(*servo_speed)--;
						}
						
						// F4 Decrease Speed
						if ((dirf_changes&(1<<3)))
						{
							if (*servo_speed<255)
							(*servo_speed)++;
						}
						
						// F0 Save Config
						if (dirf_changes&(1<<4))
						{
							eeprom.data.servo_min[servo_index] = servo[servo_index].min;
							eeprom.data.servo_max[servo_index] = servo[servo_index].max;
							
							eeprom.data.servo_time_ratio[servo_index] = servo[servo_index].time_ratio;
							
							eeprom.data.servo_timeout = servo_timeout_shadow;
							eeprom_sync_storage();
							eeprom.data.servo_timeout = 0;
							
							// Reset state machine, allow selection of next servo
							servo_config_state = 0;
							// Disable power in the meantime
							if (servo_config_mode_active!=0)
							{
								eeprom.data.servo_timeout = servo_timeout_shadow;
								servo_config_mode_active = 0;
							}
						}
						
						break;
				}
				
				if (dirf_changes)
				{
					sendLocoNet4BytePacket(OPC_LOCO_SPD, rSlot.slot, 1);
				}
					
				rSlot.dirf = LnPacket->ldf.dirf;

			}
			break;
		case OPC_LOCO_SPD:
			if ((LnPacket->lsp.slot==rSlot.slot) && (servo_config_state!=0))
			{
				servo_config_mode_timeout = clock_time();
				if (LnPacket->lsp.spd==1)
				{
					servo_base = *servo_act;
				}
				else if (LnPacket->lsp.spd>1)
				{
					int32_t temp32 = 0;
					switch (servo_mult)
					{
						case 1:
							temp32 = (int32_t) servo_base + (LnPacket->lsp.spd - 1)*256;
							break;
						case 2:
							temp32 = (int32_t) servo_base + (LnPacket->lsp.spd - 1)*32;
							break;
						case 3:
							temp32 = (int32_t) servo_base + (LnPacket->lsp.spd - 1);
							break;
						case -1:
							temp32 = (int32_t) servo_base - (LnPacket->lsp.spd - 1)*256;
							break;
						case -2:
							temp32 = (int32_t) servo_base - (LnPacket->lsp.spd - 1)*32;
							break;
						case -3:
							temp32 = (int32_t) servo_base - (LnPacket->lsp.spd - 1);
							break;
					}
						
					if (temp32<0)
						*servo_act = 0;
					else if (temp32>65535)
						*servo_act = 65535;
					else
						*servo_act = temp32;
				}
				else
				{
					*servo_act = servo_base;
				}
			}
			break;
		case OPC_LOCO_SND:
			if ((LnPacket->ls.slot==rSlot.slot) && (servo_config_state!=0))
			{
				servo_config_mode_timeout = clock_time();
				uint8_t snd_changes = rSlot.snd^LnPacket->ls.snd;
						
				// F5 Center servo (current position)
				if (snd_changes&(1<<0))
				{
					*servo_act = 32767;
					servo_base = *servo_act;
					
				}
				
				// F6 Load defaults: Center servo (both positions) and reset speed
				if (snd_changes&(1<<1))
				{
					servo[servo_index].min = 32767;
					servo[servo_index].max = 32768;
					servo[servo_index].time_ratio = 16;
					servo_base = *servo_act;				
				}
				
				rSlot.snd = LnPacket->ls.snd;
			}
			break;
	}
}

void ln_sv_cmd_callback(uint8_t cmd)
{
	static uint16_t * servo_act[] = {&servo[0].min, &servo[1].min};
	
	if (cmd==5)
	{
		rSlot.slot = 0;
		servo_config_state = 0;
		if (servo_config_mode_active!=0)
		{
			eeprom.data.servo_timeout = servo_timeout_shadow;
			servo_config_mode_active = 0;
		}
		else
		{
			servo_timeout_shadow = eeprom.data.servo_timeout;
			eeprom.data.servo_timeout = 0;
			servo_config_mode_active = 1;
		}
	}
	else if ((cmd>=10) && (cmd<30))
	{
		uint8_t idx_servo;
		uint8_t update_pos = 0;
		int32_t temp32;
		
		if (cmd<20)
		{
			cmd -= 10;
			idx_servo = 0;
		}
		else
		{
			cmd -= 20;
			idx_servo = 1;
		}
		
		switch (cmd)
		{
			// "Page up": big increment
			case 9:
				temp32 = (int32_t) *servo_act[idx_servo] + 1024;
				update_pos = 1;
				break;
			// "Page down": big decrement
			case 3:
				temp32 = (int32_t) *servo_act[idx_servo] - 1024;
				update_pos = 1;
				break;
			// "Up": medium increment
			case 8:
				temp32 = (int32_t) *servo_act[idx_servo] + 256;
				update_pos = 1;
				break;
			// "Down": medium decrement
			case 2:
				temp32 = (int32_t) *servo_act[idx_servo] - 256;
				update_pos = 1;
				break;
			// Small increment
			case 7:
				temp32 = (int32_t) *servo_act[idx_servo] + 32;
				update_pos = 1;
				break;
			// Small decrement
			case 1:
				temp32 = (int32_t) *servo_act[idx_servo] - 32;
				update_pos = 1;
				break;
			// "Left": position left
			case 4:
				ln_gpio_status[0] &= ~(1<<idx_servo);
				ln_gpio_status_pre[0] &= ~(1<<idx_servo);
				ln_gpio_status_flag[0] |= (1<<idx_servo);
				servo_act[idx_servo] = &servo[idx_servo].min;
				break;
			// "Right": position right
			case 6:
				ln_gpio_status[0] |= (1<<idx_servo);
				ln_gpio_status_pre[0] |= (1<<idx_servo);
				ln_gpio_status_flag[0] |= (1<<idx_servo);
				servo_act[idx_servo] = &servo[idx_servo].max;
				break;
			// Save settings
			case 0:
				eeprom.data.servo_min[idx_servo] = servo[idx_servo].min;
				eeprom.data.servo_max[idx_servo] = servo[idx_servo].max;
				
				eeprom.data.servo_timeout = servo_timeout_shadow;				
				eeprom_sync_storage();
				eeprom.data.servo_timeout = 0;
				
				break;
			// Center servo in current position (left or right)
			case 5:
				*servo_act[idx_servo] = 32767;
				break;
		}
		
		if (update_pos==0)
			return;
		
		if (temp32<0)
			*servo_act[idx_servo] = 0;
		else if (temp32>65535)
			*servo_act[idx_servo] = 65535;
		else
			*servo_act[idx_servo] = temp32;
	}
}

void relay_governor(void)
{
	uint8_t relay_change;
	
	if (relay_cmd!=0)
		return;
	
	relay_change = relay_request^relay_state;
	if (relay_change)
	{
		if (relay_change&(1<<0))
		{
			if (relay_request&(1<<0))
			{
				relay_cmd |= RELAY_CMD_RIGHT1;
			}
			else
			{
				relay_cmd |= RELAY_CMD_LEFT1;
			}
		}
		
		if (relay_change&(1<<1))
		{
			if (relay_request&(1<<1))
			{
				relay_cmd |= RELAY_CMD_RIGHT2;
			}
			else
			{
				relay_cmd |= RELAY_CMD_LEFT2;
			}
		}
		
		relay_cmd |= 0xF0;
		relay_state = relay_request;
		eeprom_status.data.relay_request = relay_request;
	}
}

#define RELAY_CHECK_DONE(RCMD)	\
if ((relay_cmd&0xF0)==0)	\
{ \
	relay_cmd &= ~(RCMD); \
	relay_cmd |= 0xF0; \
	\
	/* Idle State */ \
	port_user &= ~((1<<PU_RELAY_RC1)|(1<<PU_RELAY_RC2)|(1<<PU_RELAY_RC3)|(1<<PU_RELAY_RC4)); \
} \

PROCESS_THREAD(relay_process, ev, data)
{
	
	PROCESS_BEGIN();
	
	// Initialization
	relay_request = eeprom_status.data.relay_request;
	relay_state = !relay_request;
	etimer_set(&relay_timer, 20E-3*CLOCK_SECOND);
	
	while (1)
	{
		relay_governor();
		
		if ((relay_cmd&0xF)==0)
		{
			// Idle state
			port_user &= ~((1<<PU_RELAY_RC1)|(1<<PU_RELAY_RC2)|(1<<PU_RELAY_RC3)|(1<<PU_RELAY_RC4));
		}
		
		if (relay_cmd&RELAY_CMD_RIGHT1)
		{
			port_user |= (1<<PU_RELAY_1)|(1<<PU_RELAY_RC2)|(1<<PU_RELAY_RC3);
			RELAY_CHECK_DONE(RELAY_CMD_RIGHT1);
		}
		else if (relay_cmd&RELAY_CMD_LEFT1)
		{
			port_user &= ~(1<<PU_RELAY_1);
			port_user |= (1<<PU_RELAY_RC1)|(1<<PU_RELAY_RC4);
			RELAY_CHECK_DONE(RELAY_CMD_LEFT1);
		}
		else if (relay_cmd&RELAY_CMD_RIGHT2)
		{
			port_user |= (1<<PU_RELAY_2)|(1<<PU_RELAY_RC3)|(1<<PU_RELAY_RC4);
			RELAY_CHECK_DONE(RELAY_CMD_RIGHT2);
		}
		else if (relay_cmd&RELAY_CMD_LEFT2)
		{
			port_user &= ~(1<<PU_RELAY_2);
			port_user |= (1<<PU_RELAY_RC1)|(1<<PU_RELAY_RC2);
			RELAY_CHECK_DONE(RELAY_CMD_LEFT2);
		}

		if ((relay_cmd&0xF0) == 0)
			relay_cmd = 0;
		else
			relay_cmd -= 0x10;
		
		PROCESS_YIELD();
		etimer_reset(&relay_timer);
	}
	
	PROCESS_END();
}

PROCESS_THREAD(signal_process, ev, data)
{
	static uint8_t signal_mapping[8];
	static uint8_t signal1_mask = 0;
	static uint8_t signal2_mask = 0;
	uint8_t config_mask = 0;
	
	PROCESS_BEGIN();
	
	// Initialization
	
	// Create mapping port_user bit <-> pwm_port index based on mux configuration
	// Mux 0 has highest priority and thus comes last
	for (uint8_t index=0;index<16;index++)
	{
		if ((eeprom.data.port_map_mux2[index]>=(112+8)) && (eeprom.data.port_map_mux2[index]<(112+16)))
		{
			uint8_t bit = eeprom.data.port_map_mux2[index] - (112+8);
			signal_mapping[bit] = index;
			config_mask |= (1<<bit);
		}
		
		if ((eeprom.data.port_map_mux1[index]>=(112+8)) && (eeprom.data.port_map_mux1[index]<(112+16)))
		{
			uint8_t bit = eeprom.data.port_map_mux1[index] - (112+8);
			signal_mapping[bit] = index;
			config_mask |= (1<<bit);
		}
		
		if ((eeprom.data.port_map_mux0[index]>=(112+8)) && (eeprom.data.port_map_mux0[index]<(112+16)))
		{
			uint8_t bit = eeprom.data.port_map_mux0[index] - (112+8);
			signal_mapping[bit] = index;
			config_mask |= (1<<bit);
		}
	}
	
	// Determine the outputs that are used by the respective signal
	for (uint8_t index=0;index<4;index++)
	{
		signal1_mask |= eeprom.data.signal_aspect[0][index];
		signal2_mask |= eeprom.data.signal_aspect[1][index];
	}
	
	// Ignore not mapped outputs
	signal1_mask &= config_mask;
	signal2_mask &= config_mask;
	
	// If signal1 is configured, set first aspect regardless of eeprom status
	if (signal1_mask!=0)
	{
		ln_gpio_status[1] &= ~((1<<0)|(1<<1));
	}
	
	// If signal2 is configured, set first aspect regardless of eeprom status
	if (signal2_mask!=0)
	{
		ln_gpio_status[1] &= ~((1<<2)|(1<<3));
	}
	
	// Apply first signal aspect (usually stop)
	port_user = (port_user&0xFF)|(((eeprom.data.signal_aspect[0][0]|eeprom.data.signal_aspect[1][0]))<<8);
	
	while (1)
	{
		PROCESS_PAUSE();
		
		// Signal 1
		// Check if a new command was received
		if (ln_gpio_status_flag[1]&(1<<0))
		{
			// All outputs off
			port_user &= ~(signal1_mask<<8);
			// Check that all outputs are off before applying new outputs
			if ((!(signal1_mask&(1<<0)) || (pwm_port[signal_mapping[0]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[0]])) &&
				(!(signal1_mask&(1<<1)) || (pwm_port[signal_mapping[1]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[1]])) &&
				(!(signal1_mask&(1<<2)) || (pwm_port[signal_mapping[2]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[2]])) &&
				(!(signal1_mask&(1<<3)) || (pwm_port[signal_mapping[3]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[3]])) &&
				(!(signal1_mask&(1<<4)) || (pwm_port[signal_mapping[4]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[4]])) &&
				(!(signal1_mask&(1<<5)) || (pwm_port[signal_mapping[5]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[5]])) &&
				(!(signal1_mask&(1<<6)) || (pwm_port[signal_mapping[6]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[6]])) &&
				(!(signal1_mask&(1<<7)) || (pwm_port[signal_mapping[7]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[7]])))
			{
				// Clear flag
				ln_gpio_status_flag[1] &= ~(1<<0);
				// Set new outputs accordingly
				if (ln_gpio_status[1]&(1<<0))
				{
					port_user |= eeprom.data.signal_aspect[0][1]<<8;
				}
				else
				{
					port_user |= eeprom.data.signal_aspect[0][0]<<8;
				}
			}
		}
		else if (ln_gpio_status_flag[1]&(1<<1))
		{
			// All outputs off
			port_user &= ~(signal1_mask<<8);
			// Check that all outputs are off before applying new outputs
			if ((!(signal1_mask&(1<<0)) || (pwm_port[signal_mapping[0]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[0]])) &&
				(!(signal1_mask&(1<<1)) || (pwm_port[signal_mapping[1]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[1]])) &&
				(!(signal1_mask&(1<<2)) || (pwm_port[signal_mapping[2]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[2]])) &&
				(!(signal1_mask&(1<<3)) || (pwm_port[signal_mapping[3]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[3]])) &&
				(!(signal1_mask&(1<<4)) || (pwm_port[signal_mapping[4]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[4]])) &&
				(!(signal1_mask&(1<<5)) || (pwm_port[signal_mapping[5]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[5]])) &&
				(!(signal1_mask&(1<<6)) || (pwm_port[signal_mapping[6]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[6]])) &&
				(!(signal1_mask&(1<<7)) || (pwm_port[signal_mapping[7]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[7]])))
			{
				// Clear flag
				ln_gpio_status_flag[1] &= ~(1<<1);
				// Set new outputs accordingly
				if (ln_gpio_status[1]&(1<<1))
				{
					port_user |= eeprom.data.signal_aspect[0][3]<<8;
				}
				else
				{
					port_user |= eeprom.data.signal_aspect[0][2]<<8;
				}
			}
		}
		
		// Signal 2
		// Check if a new command was received
		if (ln_gpio_status_flag[1]&(1<<2))
		{
			port_user &= ~(signal2_mask<<8);
			// Check that all outputs are off before applying new outputs
			if ((!(signal2_mask&(1<<0)) || (pwm_port[signal_mapping[0]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[0]])) &&
				(!(signal2_mask&(1<<1)) || (pwm_port[signal_mapping[1]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[1]])) &&
				(!(signal2_mask&(1<<2)) || (pwm_port[signal_mapping[2]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[2]])) &&
				(!(signal2_mask&(1<<3)) || (pwm_port[signal_mapping[3]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[3]])) &&
				(!(signal2_mask&(1<<4)) || (pwm_port[signal_mapping[4]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[4]])) &&
				(!(signal2_mask&(1<<5)) || (pwm_port[signal_mapping[5]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[5]])) &&
				(!(signal2_mask&(1<<6)) || (pwm_port[signal_mapping[6]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[6]])) &&
				(!(signal2_mask&(1<<7)) || (pwm_port[signal_mapping[7]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[7]])))
			{
				// Clear flag
				ln_gpio_status_flag[1] &= ~(1<<2);
				// Set new outputs accordingly
				if (ln_gpio_status[1]&(1<<2))
				{
					port_user |= eeprom.data.signal_aspect[1][1]<<8;
				}
				else
				{
					port_user |= eeprom.data.signal_aspect[1][0]<<8;
				}
			}
		}
		else if (ln_gpio_status_flag[1]&(1<<3))
		{
			port_user &= ~(signal2_mask<<8);
			// Check that all outputs are off before applying new outputs
			if ((!(signal2_mask&(1<<0)) || (pwm_port[signal_mapping[0]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[0]])) &&
				(!(signal2_mask&(1<<1)) || (pwm_port[signal_mapping[1]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[1]])) &&
				(!(signal2_mask&(1<<2)) || (pwm_port[signal_mapping[2]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[2]])) &&
				(!(signal2_mask&(1<<3)) || (pwm_port[signal_mapping[3]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[3]])) &&
				(!(signal2_mask&(1<<4)) || (pwm_port[signal_mapping[4]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[4]])) &&
				(!(signal2_mask&(1<<5)) || (pwm_port[signal_mapping[5]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[5]])) &&
				(!(signal2_mask&(1<<6)) || (pwm_port[signal_mapping[6]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[6]])) &&
				(!(signal2_mask&(1<<7)) || (pwm_port[signal_mapping[7]].dimm_current==eeprom.data.port_brightness_off[signal_mapping[7]])))
			{
				// Clear flag
				ln_gpio_status_flag[1] &= ~(1<<3);
				// Set new outputs accordingly
				if (ln_gpio_status[1]&(1<<3))
				{
					port_user |= eeprom.data.signal_aspect[1][3]<<8;
				}
				else
				{
					port_user |= eeprom.data.signal_aspect[1][2]<<8;
				}
			}
		}	
	}
	
	PROCESS_END();
}

inline void servo_power_enable(void)
{
	port_user |= (1<<PU_SERVO_POWER);
}

inline void servo_power_disable(void)
{
	port_user &= ~(1<<PU_SERVO_POWER);
}

#define PORT_PIN0	(PORTC.IN&(1<<2)) // SIG_S12_PWMPC2
#define PORT_PIN1	(PORTC.IN&(1<<3)) // SIG_S22_PWMPC3
#define PORT_PIN2	(PORTC.IN&(1<<7)) // SIG_IS1_RX	PC7
#define PORT_PIN3	(PORTC.IN&(1<<6)) // SIG_IS2_RX	PC6
#define PORT_PIN4	(PORTD.IN&(1<<5)) // SIG_IS1_TX	PD5
#define PORT_PIN5	(PORTC.IN&(1<<5)) // SIG_IS2_TX	PC5
#define PORT_PIN6	(PORTD.IN&(1<<1)) // SIG_IS1_O	PD1
#define PORT_PIN7	(PORTD.IN&(1<<0)) // SIG_IS2_O	PD0
#define PORT_PIN8	(PORTA.IN&(1<<6)) // S1L - PA6
#define PORT_PIN9	(PORTA.IN&(1<<7)) // S1R - PA7
#define PORT_PIN10	(PORTB.IN&(1<<0)) // S1H - PB0
#define PORT_PIN11	(PORTB.IN&(1<<1)) // S2L - PB1
#define PORT_PIN12	(PORTC.IN&(1<<0)) // S2R - PC0
#define PORT_PIN13	(PORTC.IN&(1<<1)) // S2H - PC1
#define PORT_PIN14	(PORTA.IN&(1<<5)) // SERVO_POWER PA5
#define PORT_PIN15	(0)				  // n/a

uint16_t port_pin_status(void)
{
	uint16_t temp16;
	temp16 = PORT_PIN0?1:0;
	temp16 |= PORT_PIN1?2:0;
	temp16 |= PORT_PIN2?4:0;
	temp16 |= PORT_PIN3?8:0;
	temp16 |= PORT_PIN4?16:0;
	temp16 |= PORT_PIN5?32:0;
	temp16 |= PORT_PIN6?64:0;
	temp16 |= PORT_PIN7?128:0;
	temp16 |= PORT_PIN8?256:0;
	temp16 |= PORT_PIN9?512:0;
	temp16 |= PORT_PIN10?1024:0;
	temp16 |= PORT_PIN11?2048:0;
	temp16 |= PORT_PIN12?4096:0;
	temp16 |= PORT_PIN13?8192:0;
	temp16 |= PORT_PIN14?16384:0;
	temp16 |= PORT_PIN15?32768:0;
	
	return temp16;
}

void port_di_init(void)
{
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 0, 2);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 1, 3);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 2, 7);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 3, 6);
	MAP_BITS(eeprom.data.port_dir, PORTD.DIR, 4, 5);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 5, 5);
	MAP_BITS(eeprom.data.port_dir, PORTD.DIR, 6, 1);
	MAP_BITS(eeprom.data.port_dir, PORTD.DIR, 7, 0);
	MAP_BITS(eeprom.data.port_dir, PORTA.DIR, 8, 6);
	MAP_BITS(eeprom.data.port_dir, PORTA.DIR, 9, 7);
	MAP_BITS(eeprom.data.port_dir, PORTB.DIR, 10, 0);
	MAP_BITS(eeprom.data.port_dir, PORTB.DIR, 11, 1);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 12, 0);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 13, 1);
	MAP_BITS(eeprom.data.port_dir, PORTA.DIR, 14, 5);

	if (eeprom.data.port_config&(1<<PORT_MODE_PULLUP_ENABLE))
	{
		PORTC.PIN7CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN5CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN5CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN1CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN0CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN0CTRL = (PORTC.PIN0CTRL&(~PORT_OPC_gm))|PORT_OPC_PULLUP_gc;
		PORTC.PIN1CTRL = (PORTC.PIN1CTRL&(~PORT_OPC_gm))|PORT_OPC_PULLUP_gc;
		
		PORTA.PIN5CTRL = PORT_OPC_PULLUP_gc;
		PORTA.PIN6CTRL = PORT_OPC_PULLUP_gc;
		PORTA.PIN7CTRL = PORT_OPC_PULLUP_gc;
		PORTB.PIN0CTRL = PORT_OPC_PULLUP_gc;
		PORTB.PIN1CTRL = PORT_OPC_PULLUP_gc;
	}
	else
	{
		PORTC.PIN7CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN6CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN5CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN5CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN1CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN0CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN0CTRL = (PORTC.PIN0CTRL&(~PORT_OPC_gm))|PORT_OPC_TOTEM_gc;
		PORTC.PIN1CTRL = (PORTC.PIN1CTRL&(~PORT_OPC_gm))|PORT_OPC_TOTEM_gc;

		PORTA.PIN5CTRL = PORT_OPC_TOTEM_gc;
		PORTA.PIN6CTRL = PORT_OPC_TOTEM_gc;
		PORTA.PIN7CTRL = PORT_OPC_TOTEM_gc;
		PORTB.PIN0CTRL = PORT_OPC_TOTEM_gc;
		PORTB.PIN1CTRL = PORT_OPC_TOTEM_gc;
	}
	
	// Initial key state
	port_di = port_pin_status();
	
	// Restore command bits of relay 2
	if (eeprom_status.data.relay_request&(1<<1))
	{
		port_mapped |= (1<<15);
		port_user |= (1<<PU_RELAY2_REQ);
	}
	else
	{
		port_mapped &= ~(1<<15);
		port_user &= ~(1<<PU_RELAY2_REQ);
	}
}

void port_update_mapping(void)
{
	port[2][2] = pwm_port[0].pwm_current;
	port[2][3] = pwm_port[1].pwm_current;
	port[2][7] = pwm_port[2].pwm_current;
	port[2][6] = pwm_port[3].pwm_current;
	port[3][5] = pwm_port[4].pwm_current;
	port[2][5] = pwm_port[5].pwm_current;
	port[3][1] = pwm_port[6].pwm_current;
	port[3][0] = pwm_port[7].pwm_current;
	port[0][6] = pwm_port[8].pwm_current;
	port[0][7] = pwm_port[9].pwm_current;
	port[1][0] = pwm_port[10].pwm_current;
	port[1][1] = pwm_port[11].pwm_current;
	port[2][0] = pwm_port[12].pwm_current;
	port[2][1] = pwm_port[13].pwm_current;
	port[0][5] = pwm_port[14].pwm_current;
	
	if (port_mapped&(1<<15))
	{
		relay_request |= (1<<1);
	}
	else
	{
		relay_request &= ~(1<<1);
	}
}
