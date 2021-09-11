/*
 * Copyright (c) 2021, Manuel Vetterli
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
#include <util/delay.h>
#include "sys/process.h"
#include "sys/etimer.h"
#include "gbm12.h"
#include "config.h"
#include "eeprom.h"

PROCESS(gbm12_process,"GBM12 Application Process");

const uint8_t track2adc[] = {11,10,9,8,7,6,5,4,3,2,1,0};

PROCESS_THREAD(gbm12_process, ev, data)
{
	
	PROCESS_BEGIN();
	
	// Initialization
	gbm_init();
	process_start(&gbm_process, NULL);
		
	while (1)
	{
		PROCESS_PAUSE();
		
		PROCESS_EXIT();
	}
	
	PROCESS_END();
}

#define PORT_PIN0	(0) // P1
#define PORT_PIN1	(0) // P2
#define PORT_PIN2	(0) // P3
#define PORT_PIN3	(0) // P4
#define PORT_PIN4	(0) // P5
#define PORT_PIN5	(0) // P6
#define PORT_PIN6	(0) // P7
#define PORT_PIN7	(0) // P8

#define PORT_PIN8	(0) // P9
#define PORT_PIN9	(0) // P10
#define PORT_PIN10	(0) // P11
#define PORT_PIN11	(0) // P12
#define PORT_PIN12	(0) // P13
#define PORT_PIN13	(0) // P14
#define PORT_PIN14	(0) // P15
#define PORT_PIN15	(0) // P16

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
	// Optional "DCC" input for ADC synchronization
	PORTE.PIN0CTRL = PORT_ISC_BOTHEDGES_gc | PORT_OPC_PULLDOWN_gc;
	EVSYS.CH4MUX = EVSYS_CHMUX_PORTE_PIN0_gc;

	if (eeprom.data.port_config&(1<<PORT_MODE_PULLUP_ENABLE))
	{
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
	
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 0, 3);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 1, 4);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 2, 7);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 3, 6);
	MAP_BITS(eeprom.data.port_dir, PORTD.DIR, 4, 5);
	MAP_BITS(eeprom.data.port_dir, PORTD.DIR, 5, 4);
	MAP_BITS(eeprom.data.port_dir, PORTD.DIR, 6, 1);
	MAP_BITS(eeprom.data.port_dir, PORTD.DIR, 7, 0);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 8, 5);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 9, 2);
	MAP_BITS(eeprom.data.port_dir, PORTD.DIR, 10, 2);
	MAP_BITS(eeprom.data.port_dir, PORTD.DIR, 11, 3);
	
	// Initial key state
	port_di = port_pin_status();
}

void port_update_mapping(void)
{
	port[2][3] = pwm_port[0].pwm_current;
	port[2][4] = pwm_port[1].pwm_current;
	port[2][7] = pwm_port[2].pwm_current;
	port[2][6] = pwm_port[3].pwm_current;
	port[3][5] = pwm_port[4].pwm_current;
	port[3][4] = pwm_port[5].pwm_current;
	port[3][1] = pwm_port[6].pwm_current;
	port[3][0] = pwm_port[7].pwm_current;
	port[2][5] = pwm_port[8].pwm_current;
	port[2][2] = pwm_port[9].pwm_current;
	port[3][2] = pwm_port[10].pwm_current;
	port[3][3] = pwm_port[11].pwm_current;
}
