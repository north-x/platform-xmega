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

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <stdlib.h>
#include "sys/process.h"
#include "sys/etimer.h"
#include "ln_interface.h"
#include "gbm.h"
#include "config.h"
#include "eeprom.h"
#include "port.h"

static struct etimer gbm_timer;
volatile uint8_t gbm_adc_phase;
int8_t gbm_value_act[12];
uint8_t gbm_avg[12];
uint16_t gbm_avg_int[12];

uint8_t gbm_filter_cnt[12];
uint16_t gbm_register_filt;
uint16_t gbm_register_filt_filt;

uint8_t gbm_track_select_L;
uint8_t gbm_track_select_H;
uint8_t gbm_temp_multi;


PROCESS(gbm_process, "GBM");

PROCESS_THREAD(gbm_process, ev, data)
{
	static uint8_t track = 0;
	static uint8_t cnt = 0;
	PROCESS_BEGIN();
	
	// Initialization
	etimer_set(&gbm_timer, 1E-3*CLOCK_SECOND);
	
	while (1)
	{
		PROCESS_YIELD();
		for (track=0;track<12;track++)
		{
			gbm_avg[track] = gbm_avg_int[track] / GBM_TIMECONST;
			gbm_avg_int[track] += abs(gbm_value_act[track2adc[track]]+1);
			gbm_avg_int[track] -= gbm_avg[track];
			
			if (gbm_avg[track] > eeprom.data.gbm_threshold_on[track])
			{
				gbm_register_filt |= (1<<track);
			}
			else if (gbm_avg[track] < eeprom.data.gbm_threshold_off[track])
			{
				gbm_register_filt &= ~(1<<track);
			}
		}
		
		etimer_reset(&gbm_timer);
		cnt++;
		
		if (cnt>=50)
		{
			cnt = 0;
		
			for (track=0;track<12;track++)
			{
				if (gbm_register_filt&(1<<track))
				{
					if ((gbm_register_filt_filt&(1<<track))==0)
					{
						gbm_filter_cnt[track]++;
						if (gbm_filter_cnt[track]>eeprom.data.gbm_delay_on[track])
						{
							gbm_register_filt_filt |= (1<<track);
							gbm_filter_cnt[track] = eeprom.data.gbm_delay_off[track];
						}
					}
					else
					{
						gbm_filter_cnt[track] = eeprom.data.gbm_delay_off[track];
					}
				
				}
				else
				{
					if ((gbm_register_filt_filt&(1<<track))!=0)
					{
						if (gbm_filter_cnt[track]==0)
							gbm_register_filt_filt &= ~(1<<track);
						else
							gbm_filter_cnt[track]--;
					}
					else
					{
						gbm_filter_cnt[track] = 0;
					}
				}
			}
			
			port_user = gbm_register_filt_filt;
		}
	}
	
	PROCESS_END();
}

void gbm_init(void)
{
	// Disable input on all analog pins
	PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;	
	PORTA.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
	PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
	
	// Counter counts from 0 to PER
	// - if DCC input (event ch4) changes, timer is reset
	// - if counter reaches CCB, ADC is triggered (via event ch3)
	TCC0.PER = 3200; //100*F_CPU/1E6;
	TCC0.CCB = 1440; //50*F_CPU/1E6;
	TCC0.CTRLB = TC_WGMODE_NORMAL_gc;
	TCC0.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH4_gc;
	
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
#if 0
	TCC0.INTCTRLB = TC_CCBINTLVL_HI_gc; // Enable CCB interrupt for debugging
	PORTC.DIRSET = (1<<7); // debug output GBM 1
#endif	

	EVSYS.CH3MUX = EVSYS_CHMUX_TCC0_CCB_gc;
	
	//Load ADC calibration from production signature
	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	ADCA.CALL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	ADCA.CALH = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;

	ADCA.PRESCALER  = ADC_PRESCALER_DIV16_gc; 	// 2 MSPS: 32 MHz / 16
	ADCA.CTRLB      = ADC_RESOLUTION_LEFT12BIT_gc | ADC_CONMODE_bm;     // signed mode, 12 bit left adjusted
	ADCA.REFCTRL    = ADC_REFSEL_INT1V_gc;		// REF= 1V
	ADCA.EVCTRL     = ADC_SWEEP_0123_gc | ADC_EVACT_SWEEP_gc | ADC_EVSEL_3456_gc;     // Sweep all channels, event channel 3
	
	// Configure all channels as single-ended
	// and initialize the input multiplexer
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;
	ADCA.CH0.CTRL   = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	
	ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
	ADCA.CH1.CTRL   = ADC_CH_INPUTMODE_SINGLEENDED_gc;

	ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;
	ADCA.CH2.CTRL   = ADC_CH_INPUTMODE_SINGLEENDED_gc;

	ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
	ADCA.CH3.CTRL   = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH3.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_HI_gc;			// Enable high interrupt for ADC Channel 3
	
	ADCA.INTFLAGS = 0xF;			// Clear the interrupt flags
	ADCA.CTRLA = ADC_ENABLE_bm;		// Enable ADC
}

#if 0
ISR(TCC0_CCB_vect)
{
	PORTC.OUTSET = (1<<7); // ADC Trigger
}
#endif

ISR(ADCA_CH3_vect)
{
	if (TCC0.CNT<TCC0.CCB)
	{
		// Discard result since input signal was not stable during conversion
		return;
	}
	
	if (gbm_adc_phase==0)
	{
		// Configure MUX to scan tracks 5-8
		ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;
		ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc;
		ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN6_gc;
		ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN7_gc;
		
		// Store results
		gbm_value_act[0] = (ADCA.CH0RES>>8);
		gbm_value_act[1] = (ADCA.CH1RES>>8);
		gbm_value_act[2] = (ADCA.CH2RES>>8);
		gbm_value_act[3] = (ADCA.CH3RES>>8);
		
		gbm_adc_phase = 1;
	}
	else if (gbm_adc_phase==1)
	{
		// Configure MUX to scan tracks 9-12
		ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN8_gc;
		ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN9_gc;
		ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN10_gc;
		ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN11_gc;
		
		// Store results
		gbm_value_act[4] = (ADCA.CH0RES>>8);
		gbm_value_act[5] = (ADCA.CH1RES>>8);
		gbm_value_act[6] = (ADCA.CH2RES>>8);
		gbm_value_act[7] = (ADCA.CH3RES>>8);
		
		gbm_adc_phase = 2;
	}
	else
	{
		// Configure MUX to scan tracks 1-4
		ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;
		ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
		ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;
		ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
		
		// Store results
		gbm_value_act[8] = (ADCA.CH0RES>>8);
		gbm_value_act[9] = (ADCA.CH1RES>>8);
		gbm_value_act[10] = (ADCA.CH2RES>>8);
		gbm_value_act[11] = (ADCA.CH3RES>>8);
		
		gbm_adc_phase = 0;
	}
#if 0
	PORTC.OUTCLR = (1<<7);
#endif
}

void gbm_helper_multi_threshold_on(void)
{
	uint8_t index;
	
	for (index=0;index<8;index++)
	{
		if (gbm_track_select_L&(1<<index))
		{
			eeprom.data.gbm_threshold_on[index] = gbm_temp_multi;
		}
		
		if ((index<4) && (gbm_track_select_H&(1<<index)))
		{
			eeprom.data.gbm_threshold_on[index+8] = gbm_temp_multi;
		}
	}
}

void gbm_helper_multi_threshold_off(void)
{
	uint8_t index;
	
	for (index=0;index<8;index++)
	{
		if (gbm_track_select_L&(1<<index))
		{
			eeprom.data.gbm_threshold_off[index] = gbm_temp_multi;
		}
		if ((index<4) && (gbm_track_select_H&(1<<index)))
		{
			eeprom.data.gbm_threshold_off[index+8] = gbm_temp_multi;
		}
	}
}

void gbm_helper_multi_delay_on(void)
{
	uint8_t index;
	
	for (index=0;index<8;index++)
	{
		if (gbm_track_select_L&(1<<index))
		{
			eeprom.data.gbm_delay_on[index] = gbm_temp_multi;
		}
		if ((index<4) && (gbm_track_select_H&(1<<index)))
		{
			eeprom.data.gbm_delay_on[index+8] = gbm_temp_multi;
		}
	}
}

void gbm_helper_multi_delay_off(void)
{
	uint8_t index;
	
	for (index=0;index<8;index++)
	{
		if (gbm_track_select_L&(1<<index))
		{
			eeprom.data.gbm_delay_off[index] = gbm_temp_multi;
		}
		if ((index<4) && (gbm_track_select_H&(1<<index)))
		{
			eeprom.data.gbm_delay_off[index+8] = gbm_temp_multi;
		}
	}
}
