/*
 * Copyright (c) 2018, Manuel Vetterli
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
#include "eam.h"
#include "config.h"
#include "eeprom.h"
#include "port.h"

PROCESS(eam_process,"EAM Application Process");


PROCESS_THREAD(eam_process, ev, data)
{
	
	PROCESS_BEGIN();
	
	// Initialization
	
	while (1)
	{
		PROCESS_PAUSE();
		
	}
	
	PROCESS_END();
}

#if 1
#define PORT_PIN0	(PORTB.IN&(1<<2)) // P1
#define PORT_PIN1	(PORTC.IN&(1<<2)) // P2
#define PORT_PIN2	(PORTB.IN&(1<<3)) // P3
#define PORT_PIN3	(PORTC.IN&(1<<3)) // P4
#define PORT_PIN4	(PORTC.IN&(1<<0)) // P5
#define PORT_PIN5	(PORTC.IN&(1<<4)) // P6
#define PORT_PIN6	(PORTC.IN&(1<<1)) // P7
#define PORT_PIN7	(PORTC.IN&(1<<5)) // P8

#define PORT_PIN8	(PORTC.IN&(1<<6)) // P9
#define PORT_PIN9	(PORTD.IN&(1<<2)) // P10
#define PORT_PIN10	(PORTC.IN&(1<<7)) // P11
#define PORT_PIN11	(PORTD.IN&(1<<3)) // P12
#define PORT_PIN12	(PORTD.IN&(1<<0)) // P13
#define PORT_PIN13	(PORTD.IN&(1<<4)) // P14
#define PORT_PIN14	(PORTD.IN&(1<<1)) // P15
#define PORT_PIN15	(PORTD.IN&(1<<5)) // P16
#else
// Use Virtual Ports
#define PORT_PIN0	(VPORT1.IN&(1<<2)) // P1
#define PORT_PIN1	(VPORT2.IN&(1<<2)) // P2
#define PORT_PIN2	(VPORT1.IN&(1<<3)) // P3
#define PORT_PIN3	(VPORT2.IN&(1<<3)) // P4
#define PORT_PIN4	(VPORT2.IN&(1<<0)) // P5
#define PORT_PIN5	(VPORT2.IN&(1<<4)) // P6
#define PORT_PIN6	(VPORT2.IN&(1<<1)) // P7
#define PORT_PIN7	(VPORT2.IN&(1<<5)) // P8

#define PORT_PIN8	(VPORT2.IN&(1<<6)) // P9
#define PORT_PIN9	(VPORT3.IN&(1<<2)) // P10
#define PORT_PIN10	(VPORT2.IN&(1<<7)) // P11
#define PORT_PIN11	(VPORT3.IN&(1<<3)) // P12
#define PORT_PIN12	(VPORT3.IN&(1<<0)) // P13
#define PORT_PIN13	(VPORT3.IN&(1<<4)) // P14
#define PORT_PIN14	(VPORT3.IN&(1<<1)) // P15
#define PORT_PIN15	(VPORT3.IN&(1<<5)) // P16
#endif

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
	if (eeprom.port_config&(1<<PORT_MODE_PULLUP_ENABLE))
	{
		PORTB.PIN2CTRL = PORT_OPC_PULLUP_gc;
		PORTB.PIN3CTRL = PORT_OPC_PULLUP_gc;
		
		PORTC.PIN0CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN1CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN2CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN3CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN4CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN5CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN7CTRL = PORT_OPC_PULLUP_gc;
		
		PORTD.PIN0CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN1CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN2CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN3CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN4CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN5CTRL = PORT_OPC_PULLUP_gc;
	}
	else
	{
		PORTB.PIN2CTRL = PORT_OPC_TOTEM_gc;
		PORTB.PIN3CTRL = PORT_OPC_TOTEM_gc;
		
		PORTC.PIN0CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN1CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN2CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN3CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN4CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN5CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN6CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN7CTRL = PORT_OPC_TOTEM_gc;
		
		PORTD.PIN0CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN1CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN2CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN3CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN4CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN5CTRL = PORT_OPC_TOTEM_gc;
	}
	
	MAP_BITS(eeprom.port_dir, PORTB.DIR, 0, 2);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 1, 2);
	MAP_BITS(eeprom.port_dir, PORTB.DIR, 2, 3);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 3, 3);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 4, 0);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 5, 4);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 6, 1);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 7, 5);
	
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 8, 6);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 9, 2);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 10, 7);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 11, 3);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 12, 0);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 13, 4);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 14, 1);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 15, 5);
	
	// Initial key state
	port_di = port_pin_status();
}

void port_update_mapping(void)
{
	port[1][2] = pwm_port[0].pwm_current;
	port[2][2] = pwm_port[1].pwm_current;
	port[1][3] = pwm_port[2].pwm_current;
	port[2][3] = pwm_port[3].pwm_current;
	port[2][0] = pwm_port[4].pwm_current;
	port[2][4] = pwm_port[5].pwm_current;
	port[2][1] = pwm_port[6].pwm_current;
	port[2][5] = pwm_port[7].pwm_current;
	port[2][6] = pwm_port[8].pwm_current;
	port[3][2] = pwm_port[9].pwm_current;
	port[2][7] = pwm_port[10].pwm_current;
	port[3][3] = pwm_port[11].pwm_current;
	port[3][0] = pwm_port[12].pwm_current;
	port[3][4] = pwm_port[13].pwm_current;
	port[3][1] = pwm_port[14].pwm_current;
	port[3][5] = pwm_port[15].pwm_current;
}
