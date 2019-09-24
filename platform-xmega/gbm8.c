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
