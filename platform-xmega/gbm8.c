/*
 * Copyright (c) 2019, Manuel Vetterli
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
#include "gbm8.h"
#include "config.h"
#include "eeprom.h"

PROCESS(gbm8_process,"GBM8 Application Process");

#if defined GBM8_20121212
const uint8_t track2adc[] = {11,10,9,8,3,2,1,0,4,5,6,7};
#elif defined GBM8_20120104
const uint8_t track2adc[] = {7,6,5,4,3,2,1,0,8,9,10,11};
#endif
// GBM12:
//const uint8_t track2adc[] = {11,10,9,8,7,6,5,4,3,2,1,0};

PROCESS_THREAD(gbm8_process, ev, data)
{
	
	PROCESS_BEGIN();
	
	// Initialization
#if defined GBM8_20121212
	gbm8_hw_detect();
#endif
	
	while (1)
	{
		PROCESS_PAUSE();
		
		// Reconfigure comparator to match hardware
		if (gbm_version==3)
		{
			ACA.AC0MUXCTRL = AC_MUXPOS_PIN3_gc | AC_MUXNEG_SCALER_gc;
			eeprom.data.gbm_mode = GBM_MODE_FSZ;
		}
		else if (gbm_version==2)
		{
			ACA.AC0MUXCTRL = AC_MUXPOS_PIN5_gc | AC_MUXNEG_SCALER_gc;
		}
		
		update_gbm_mode();
		
		PROCESS_EXIT();
	}
	
	PROCESS_END();
}

uint8_t gbm_mode = GBM_MODE_INIT;
uint8_t gbm_version;

PROCESS_NAME(fsz_process);
PROCESS_NAME(sbk_in_process);
PROCESS_NAME(sbk_out_process);
PROCESS_NAME(gbm_process);

void update_gbm_mode(void)
{
	if (gbm_mode==eeprom.data.gbm_mode)
		return;

	switch (gbm_mode)
	{
		case GBM_MODE_NORMAL:
			process_exit(&gbm_process);
			PORTCFG.VPCTRLB = PORTCFG_VP02MAP_PORTC_gc | PORTCFG_VP13MAP_PORTD_gc;
			break;
		case GBM_MODE_SBK:
			process_exit(&gbm_process);
			process_exit(&sbk_out_process);
			process_exit(&sbk_in_process);
			PORTCFG.VPCTRLB = PORTCFG_VP02MAP_PORTC_gc | PORTCFG_VP13MAP_PORTD_gc;
			break;
		case GBM_MODE_FSZ:
			process_exit(&fsz_process);
			PORTCFG.VPCTRLB = PORTCFG_VP02MAP_PORTC_gc | PORTCFG_VP13MAP_PORTD_gc;
			break;
	}
	
	switch (eeprom.data.gbm_mode)
	{
		case GBM_MODE_NORMAL:
			PORTCFG.VPCTRLB = PORTCFG_VP02MAP_PORTC_gc | PORTCFG_VP13MAP_PORTD_gc;
			gbm_init();
			process_start(&gbm_process, NULL);
			break;
		case GBM_MODE_SBK:
			PORTCFG.VPCTRLB = PORTCFG_VP02MAP_PORTR_gc | PORTCFG_VP13MAP_PORTD_gc;
			gbm_init();
			sbk_init();
			process_start(&gbm_process, NULL);
			process_start(&sbk_out_process, NULL);
			process_start(&sbk_in_process, NULL);
			break;
		case GBM_MODE_FSZ:
			PORTCFG.VPCTRLB = PORTCFG_VP02MAP_PORTR_gc | PORTCFG_VP13MAP_PORTD_gc;
			process_start(&fsz_process, NULL);
			break;
	}
	
	gbm_mode = eeprom.data.gbm_mode;
}

/**
 * Detect board version
 *
 * This function automatically detects the available LN communication
 * hardware and initializes the needed peripherals.
 *
 * Idea: Enable pullup on LN_RX lines, if threshold is not reached
 * the pin is a GBM input
 */
void gbm8_hw_detect(void)
{
	ACA.CTRLB = 14; // 750mV Threshold
	
	PORTA.PIN3CTRL = PORT_OPC_PULLUP_gc;
	PORTA.PIN5CTRL = PORT_OPC_PULLUP_gc;
	
	// connect comparators to pins
	ACA.AC0MUXCTRL = AC_MUXPOS_PIN3_gc | AC_MUXNEG_SCALER_gc;
	ACA.AC1MUXCTRL = AC_MUXPOS_PIN5_gc | AC_MUXNEG_SCALER_gc;
	
	// Enable comparator
	ACA.AC0CTRL = AC_ENABLE_bm | AC_INTLVL_OFF_gc | AC_HYSMODE_SMALL_gc;
	ACA.AC1CTRL = AC_ENABLE_bm | AC_INTLVL_OFF_gc | AC_HYSMODE_SMALL_gc;
	_delay_ms(1);
	/*
	switch ((ACA.STATUS>>AC_AC0STATE_bp)&3)
	{
		case 0:
			// Board is a GBM8 20120104 (not supported)
		case 1:
			// not valid
		case 2:
			// Board is a GBM8 20121212
		case 3:
			// Board is an FSZ
	}
	*/
	gbm_version = (ACA.STATUS>>AC_AC0STATE_bp)&3;
	
	ACA.AC0CTRL = 0;
	ACA.AC1CTRL = 0;
	
	PORTA.PIN3CTRL = PORT_OPC_TOTEM_gc;
	PORTA.PIN5CTRL = PORT_OPC_TOTEM_gc;
}

#define PORT_PIN0	(PORTC.IN&(1<<7)) // BM1
#define PORT_PIN1	(PORTC.IN&(1<<6)) // BM2
#define PORT_PIN2	(PORTC.IN&(1<<5)) // BM3
#define PORT_PIN3	(PORTC.IN&(1<<4)) // BM4
#define PORT_PIN4	(PORTC.IN&(1<<3)) // BM5
#define PORT_PIN5	(PORTC.IN&(1<<2)) // BM6
#define PORT_PIN6	(PORTC.IN&(1<<1)) // BM7
#define PORT_PIN7	(PORTC.IN&(1<<0)) // BM8

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
	PORTD.PIN5CTRL = PORT_ISC_BOTHEDGES_gc | PORT_OPC_PULLDOWN_gc;
	EVSYS.CH4MUX = EVSYS_CHMUX_PORTD_PIN5_gc;

	if (eeprom.data.port_config&(1<<PORT_MODE_PULLUP_ENABLE))
	{
		PORTC.PIN0CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN1CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN2CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN3CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN4CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN5CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN7CTRL = PORT_OPC_PULLUP_gc;
	}
	else
	{
		PORTC.PIN0CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN1CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN2CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN3CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN4CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN5CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN6CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN7CTRL = PORT_OPC_TOTEM_gc;
	}
	
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 0, 7);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 1, 6);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 2, 5);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 3, 4);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 4, 3);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 5, 2);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 6, 1);
	MAP_BITS(eeprom.data.port_dir, PORTC.DIR, 7, 0);
	
	// Initial key state
	port_di = port_pin_status();
}

void port_update_mapping(void)
{
	port[2][7] = pwm_port[0].pwm_current;
	port[2][6] = pwm_port[1].pwm_current;
	port[2][5] = pwm_port[2].pwm_current;
	port[2][4] = pwm_port[3].pwm_current;
	port[2][3] = pwm_port[4].pwm_current;
	port[2][2] = pwm_port[5].pwm_current;
	port[2][1] = pwm_port[6].pwm_current;
	port[2][0] = pwm_port[7].pwm_current;
}
