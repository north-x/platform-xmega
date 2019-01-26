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
#include "sys/process.h"
#include "sys/etimer.h"
#include "ln_interface.h"
#include "wa2.h"
#include "config.h"
#include "eeprom.h"

PROCESS(wa2_process,"WA2 Application Process");
PROCESS(relay_process,"Relay Control Process");

rwSlotDataMsg rSlot;
uint8_t relay_state;
uint8_t relay_request;
uint8_t relay_cmd;

static struct etimer relay_timer;

PROCESS_THREAD(wa2_process, ev, data)
{
	
	PROCESS_BEGIN();
	
	// Initialization
	rSlot.slot = 0xFF;
	
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
		else if ((eeprom.servo_multipos_opcode&0x0F) && (ln_gpio_status_flag[(eeprom.servo_multipos_opcode&0xF)/8]&(1<<((eeprom.servo_multipos_opcode&0xF)%8))))
		{
			// Set new position accordingly
			if (ln_gpio_status[(eeprom.servo_multipos_opcode&0xF)/8]&(1<<((eeprom.servo_multipos_opcode&0xF)%8)))
			{
				servo[0].position_setpoint = eeprom.servo_multipos[0][1];
			}
			else
			{
				servo[0].position_setpoint = eeprom.servo_multipos[0][0];
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
				ln_gpio_tx[(eeprom.servo_multipos_opcode&0xF)/8] |= (1<<((eeprom.servo_multipos_opcode&0xF)%8));
				ln_gpio_status_flag[(eeprom.servo_multipos_opcode&0xF)/8] &= ~(1<<((eeprom.servo_multipos_opcode&0xF)%8));
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
				relay_request |= (1<<1);
			}
			else
			{
				relay_request &= ~(1<<1);
			}
			
			// If we reached the final position, transmit the new status
			if (servo[1].position_actual==servo[1].position_setpoint)
			{
				ln_gpio_tx[0] |= (1<<1);
				ln_gpio_status_flag[0] &= ~(1<<1);
			}
			
		}
		else if ((eeprom.servo_multipos_opcode&0xF0) && (ln_gpio_status_flag[(eeprom.servo_multipos_opcode>>4)/8]&(1<<((eeprom.servo_multipos_opcode>>4)%8))))
		{
			// Set new position accordingly
			if (ln_gpio_status[(eeprom.servo_multipos_opcode>>4)/8]&(1<<((eeprom.servo_multipos_opcode>>4)%8)))
			{
				servo[1].position_setpoint = eeprom.servo_multipos[1][1];
			}
			else
			{
				servo[1].position_setpoint = eeprom.servo_multipos[1][0];
			}
			// Turn on relay if position greater than 127
			if (servo[1].position_actual>127)
			{
				relay_request |= (1<<1);
			}
			else
			{
				relay_request &= ~(1<<1);
			}
			
			// If we reached the final position, transmit a switch report
			if (servo[1].position_actual==servo[1].position_setpoint)
			{
				ln_gpio_tx[(eeprom.servo_multipos_opcode>>4)/8] |= (1<<((eeprom.servo_multipos_opcode>>4)%8));
				ln_gpio_status_flag[(eeprom.servo_multipos_opcode>>4)/8] &= ~(1<<((eeprom.servo_multipos_opcode>>4)%8));
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
	
	switch (LnPacket->data[0])
	{
		case OPC_LOCO_ADR:
			rSlot.adr = eeprom.sv_destination_id&0x7F;
			rSlot.adr2 = (eeprom.sv_destination_id>>7)&0x7F;
			rSlot.stat = LOCO_IDLE | DEC_MODE_128;
			rSlot.id1 = 0;
			rSlot.id2 = 0;

			if ((LnPacket->la.adr_hi==rSlot.adr2) && (LnPacket->la.adr_lo==rSlot.adr))
			{
				rSlot.slot = (eeprom.sv_destination_id&0x7F)?(eeprom.sv_destination_id&0x7F):1;
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
				if (rSlot.slot==0)
				{
					rSlot.slot = (eeprom.sv_destination_id&0x7F)?(eeprom.sv_destination_id&0x7F):1;
					rSlot.adr = eeprom.sv_destination_id&0x7F;
					rSlot.adr2 = (eeprom.sv_destination_id>>7)&0x7F;
				}
				
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
				uint8_t dirf_changes = rSlot.dirf^LnPacket->ldf.dirf;
					
				// Changes in DIR or F0
				if (dirf_changes&((1<<4)|(1<<5)))
				{
					// Restore "unsaved" position
					*servo_act = servo_base;
						
					// Reset multiplier
					servo_mult = 1;
						
					// Direction 0/forward = right 1/reversed=left
					if ((LnPacket->ldf.dirf&(1<<5))==0)
					{
						// F0: 0 -> Servo 1 / 1 -> Servo 2
						if (LnPacket->ldf.dirf&(1<<4))
						{
							ln_gpio_status[0] |= (1<<1);
							ln_gpio_status_pre[0] |= (1<<1);
							ln_gpio_status_flag[0] |= (1<<1);
							servo_act = &servo[1].max;
							servo_speed = &servo[1].time_ratio;
						}
						else
						{
							ln_gpio_status[0] |= (1<<0);
							ln_gpio_status_pre[0] |= (1<<0);
							ln_gpio_status_flag[0] |= (1<<0);
							servo_act = &servo[0].max;
							servo_speed = &servo[0].time_ratio;
						}
					}
					else
					{
						// F0: 0 -> Servo 1 / 1 -> Servo 2
						if (LnPacket->ldf.dirf&(1<<4))
						{
							ln_gpio_status[0] &= ~(1<<1);
							ln_gpio_status_pre[0] &= ~(1<<1);
							ln_gpio_status_flag[0] |= (1<<1);
							servo_act = &servo[1].min;
							servo_speed = &servo[1].time_ratio;
						}
						else
						{
							ln_gpio_status[0] &= ~(1<<0);
							ln_gpio_status_pre[0] &= ~(1<<0);
							ln_gpio_status_flag[0] |= (1<<0);
							servo_act = &servo[0].min;
							servo_speed = &servo[0].time_ratio;
						}
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
					
				if (dirf_changes)
				{
					//sendLocoNet4BytePacket(OPC_LOCO_DIRF, rSlot.slot, LnPacket->ldf.dirf);
					sendLocoNet4BytePacket(OPC_LOCO_SPD, rSlot.slot, 1);
				}
					
				rSlot.dirf = LnPacket->ldf.dirf;
			}
			break;
		case OPC_LOCO_SPD:
			if (LnPacket->lsp.slot==rSlot.slot)
			{
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
			if (LnPacket->ls.slot==rSlot.slot)
			{
				uint8_t snd_changes = rSlot.snd^LnPacket->ls.snd;
						
				// F5 Increase Speed
				if (snd_changes&(1<<0))
				{
					if (*servo_speed>1)
						(*servo_speed)--;
				}
				// F6 Decrease Speed
				if (snd_changes&(1<<1))
				{
					if (*servo_speed<255)
					(*servo_speed)++;
				}
				// F7 Load EEPROM Settings
				if (snd_changes&(1<<2))
				{
					servo[0].min = eeprom.servo_min[0];
					servo[1].min = eeprom.servo_min[1];
							
					servo[0].max = eeprom.servo_max[0];
					servo[1].max = eeprom.servo_max[1];
							
					servo[0].time_ratio = eeprom.servo_time_ratio[0];
					servo[1].time_ratio = eeprom.servo_time_ratio[1];
				}
				// F8 Save Config
				if (snd_changes&(1<<3))
				{
					eeprom.servo_min[0] = servo[0].min;
					eeprom.servo_min[1] = servo[1].min;
							
					eeprom.servo_max[0] = servo[0].max;
					eeprom.servo_max[1] = servo[1].max;
							
					eeprom.servo_time_ratio[0] = servo[0].time_ratio;
					eeprom.servo_time_ratio[1] = servo[1].time_ratio;
							
					eeprom_sync_storage();
				}
						
				rSlot.snd = LnPacket->ls.snd;
			}
			break;
	}
}

void ln_sv_cmd_callback(uint8_t cmd)
{
	static uint16_t * servo_act[] = {&servo[0].min, &servo[1].min};
	static uint16_t servo_timeout_shadow = 0;
	if (cmd==5)
	{
		rSlot.slot = 0;
		if (eeprom.servo_timeout==0)
		{
			eeprom.servo_timeout = servo_timeout_shadow;
		}
		else
		{
			servo_timeout_shadow = eeprom.servo_timeout;
			eeprom.servo_timeout = 0;
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
				eeprom.servo_min[idx_servo] = servo[idx_servo].min;
				eeprom.servo_max[idx_servo] = servo[idx_servo].max;
				
				eeprom.servo_timeout = servo_timeout_shadow;				
				eeprom_sync_storage();
				eeprom.servo_timeout = 0;
				
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
		eeprom_status.relay_request = relay_request;
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
	relay_request = eeprom_status.relay_request;
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
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 0, 2);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 1, 3);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 2, 7);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 3, 6);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 4, 5);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 5, 5);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 6, 1);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 7, 0);
	MAP_BITS(eeprom.port_dir, PORTA.DIR, 8, 6);
	MAP_BITS(eeprom.port_dir, PORTA.DIR, 9, 7);
	MAP_BITS(eeprom.port_dir, PORTB.DIR, 10, 0);
	MAP_BITS(eeprom.port_dir, PORTB.DIR, 11, 1);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 12, 0);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 13, 1);
	MAP_BITS(eeprom.port_dir, PORTA.DIR, 14, 5);

	if (eeprom.port_config&(1<<PORT_MODE_PULLUP_ENABLE))
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
}
