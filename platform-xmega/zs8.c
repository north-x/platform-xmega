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

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <stdlib.h>
#include "sys/process.h"
#include "sys/etimer.h"
#include "zs8.h"
#include "config.h"
#include "eeprom.h"
#include "shiftreg.h"

PROCESS(zs8_process, "ZS8 Process");

uint8_t zs_state[6] = {8, 8, 8, 8, 8, 8};
uint8_t shiftreg1, shiftreg2, shiftreg3;
uint8_t shiftreg1_shadow, shiftreg2_shadow, shiftreg3_shadow;
uint8_t zs_sr_reg[3];
uint16_t zs_port_di_z1;

volatile uint8_t adc_phase;
volatile uint8_t zs8_port_config_zs;
volatile uint8_t zs8_port_config_fp;
int8_t port_value_act[12];

void zs8_init(void)
{
	shiftreg_init();
	
	shiftreg_out(zs_sr_reg, 3, 1);
	shiftreg_enable();
	
	process_start(&zs8_process, NULL);
}

PROCESS_THREAD(zs8_process, ev, data)
{
	PROCESS_BEGIN();
	
	zs8_init();
	
	while (1)
	{
		PROCESS_PAUSE();
		
		if (eeprom.data.port_config&(1<<PORT_MODE_ZS8_INTERFACE_ENABLE))
			zs_process_port();
		
		zs_update();
		
		if ((shiftreg1_shadow!=shiftreg1) || (shiftreg2_shadow!=shiftreg2) || (shiftreg3_shadow!=shiftreg3))
		{
			shiftreg1_shadow = shiftreg1;
			shiftreg2_shadow = shiftreg2;
			shiftreg3_shadow = shiftreg3;

			zs_sr_reg[0] = shiftreg1;
			zs_sr_reg[1] = shiftreg2;
			zs_sr_reg[2] = shiftreg3;
			
			shiftreg_out(zs_sr_reg, 3, 1);			
		}
	}
	
	PROCESS_END();
}

void zs_update(void)
{
	shiftreg3 &= ~0xF0;
	switch (zs_state[0])
	{
		case 1:
		shiftreg3 |= 0;
		break;
		case 2:
		shiftreg3 |= REL4;
		break;
		case 3:
		shiftreg3 |= REL2;
		break;
		case 4:
		shiftreg3 |= REL2|REL4;
		break;
		case 5:
		shiftreg3 |= REL1;
		break;
		case 6:
		shiftreg3 |= REL1|REL3;
		break;
		case 7:
		shiftreg3 |= REL1|REL2;
		break;
		case 8:
		shiftreg3 |= REL1|REL2|REL3;
		break;
	}

	shiftreg3 &= ~0x0F;
	switch (zs_state[1])
	{
		case  1:
		shiftreg3 |= 0;
		break;
		case 2:
		shiftreg3 |= REL8;
		break;
		case 3:
		shiftreg3 |= REL6;
		break;
		case 4:
		shiftreg3 |= REL6|REL8;
		break;
		case 5:
		shiftreg3 |= REL5;
		break;
		case 6:
		shiftreg3 |= REL5|REL7;
		break;
		case 7:
		shiftreg3 |= REL5|REL6;
		break;
		case 8:
		shiftreg3 |= REL5|REL6|REL7;
		break;
	}

	shiftreg1 &= ~0x0F;
	switch (zs_state[2])
	{
		case 1:
		shiftreg1 |= 0;
		break;
		case 2:
		shiftreg1 |= REL12;
		break;
		case 3:
		shiftreg1 |= REL10;
		break;
		case 4:
		shiftreg1 |= REL10|REL12;
		break;
		case 5:
		shiftreg1 |= REL9;
		break;
		case 6:
		shiftreg1 |= REL9|REL11;
		break;
		case 7:
		shiftreg1 |= REL9|REL10;
		break;
		case 8:
		shiftreg1 |= REL9|REL10|REL11;
		break;
	}

	shiftreg2 &= ~0xF0;
	switch (zs_state[3])
	{
		case 1:
		shiftreg2 |= REL13;
		break;
		case 2:
		shiftreg2 |= REL13|REL15;
		break;
		case 3:
		shiftreg2 |= REL13|REL14;
		break;
		case 4:
		shiftreg2 |= REL13|REL14|REL15;
		break;
		case 5:
		shiftreg2 |= 0;
		break;
		case 6:
		shiftreg2 |= REL16;
		break;
		case 7:
		shiftreg2 |= REL14;
		break;
		case 8:
		shiftreg2 |= REL14|REL16;
		break;
	}

	shiftreg2 &= ~0x0F;
	switch (zs_state[4])
	{
		case 1:
		shiftreg2 |= REL17;
		break;
		case 2:
		shiftreg2 |= REL17|REL19;
		break;
		case 3:
		shiftreg2 |= REL17|REL18;
		break;
		case 4:
		shiftreg2 |= REL17|REL18|REL19;
		break;
		case 5:
		shiftreg2 |= 0;
		break;
		case 6:
		shiftreg2 |= REL20;
		break;
		case 7:
		shiftreg2 |= REL18;
		break;
		case 8:
		shiftreg2 |= REL18|REL20;
		break;
	}
	
	shiftreg1 &= ~0xF0;
	switch (zs_state[5])
	{
		case 1:
		shiftreg1 |= REL21;
		break;
		case 2:
		shiftreg1 |= REL21|REL23;
		break;
		case 3:
		shiftreg1 |= REL21|REL22;
		break;
		case 4:
		shiftreg1 |= REL21|REL22|REL23;
		break;
		case 5:
		shiftreg1 |= 0;
		break;
		case 6:
		shiftreg1 |= REL24;
		break;
		case 7:
		shiftreg1 |= REL22;
		break;
		case 8:
		shiftreg1 |= REL22|REL24;
		break;
	}
}

void zs_process_port(void)
{
	uint8_t zs_fp_addr = 0;
	uint16_t zs_port_di_rising = (zs_port_di_z1^port_mapped)&port_mapped&ZS_PORT_MASK;
	
	// decode FP address
	zs_fp_addr = port_mapped&0x7;
	
	// Check for port changes (rising edge only)
	if ((zs_port_di_rising!=0) && (zs_fp_addr!=0))
	{	
		if (port_mapped&ZS_PORT_ZS1_bm)
			zs_state[0] = zs_fp_addr;
			
		if (port_mapped&ZS_PORT_ZS2_bm)
			zs_state[1] = zs_fp_addr;
			
		if (port_mapped&ZS_PORT_ZS3_bm)
			zs_state[2] = zs_fp_addr;
			
		if (port_mapped&ZS_PORT_ZS4_bm)
			zs_state[3] = zs_fp_addr;
			
		if (port_mapped&ZS_PORT_ZS5_bm)
			zs_state[4] = zs_fp_addr;
			
		if (port_mapped&ZS_PORT_ZS6_bm)
			zs_state[5] = zs_fp_addr;
	}
	
	// Update outputs:
	// Set corresponding bit if FP address matches FP currently assigned to ZS
	if (zs_state[0]==zs_fp_addr)
		port_user |= (1<<0);
	else
		port_user &= ~(1<<0);
	
	if (zs_state[1]==zs_fp_addr)
		port_user |= (1<<1);
	else
		port_user &= ~(1<<1);
	
	if (zs_state[2]==zs_fp_addr)
		port_user |= (1<<2);
	else
		port_user &= ~(1<<2);

	if (zs_state[3]==zs_fp_addr)
		port_user |= (1<<3);
	else
		port_user &= ~(1<<3);
	
	if (zs_state[4]==zs_fp_addr)
		port_user |= (1<<4);
	else
		port_user &= ~(1<<4);
	
	if (zs_state[5]==zs_fp_addr)
		port_user |= (1<<5);
	else
		port_user &= ~(1<<5);
	
	zs_port_di_z1 = port_di;
}

uint16_t port_pin_status(void)
{
#if 0
	uint16_t temp = 0xF000;
	
	for (input=0;input<12;input++)
	{
		if (abs(port_value_act[input]+1)>100)
		{
			temp |= (1<<input);
		}
	}
#else
	uint16_t temp = 0xFE00;
	
	if (abs(port_value_act[8]+1)>100)
	{
		temp |= (1<<0);
	}
	if (abs(port_value_act[9]+1)>100)
	{
		temp |= (1<<1);
	}
	if (abs(port_value_act[10]+1)>100)
	{
		temp |= (1<<2);
	}
	if (abs(port_value_act[6]+1)>100)
	{
		temp |= (1<<3);
	}
	if (abs(port_value_act[5]+1)>100)
	{
		temp |= (1<<4);
	}
	if (abs(port_value_act[4]+1)>100)
	{
		temp |= (1<<5);
	}
	if (abs(port_value_act[2]+1)>100)
	{
		temp |= (1<<6);
	}
	if (abs(port_value_act[1]+1)>100)
	{
		temp |= (1<<7);
	}
	if (abs(port_value_act[0]+1)>100)
	{
		temp |= (1<<8);
	}
#endif

	return temp;
}

void port_di_init(void)
{
	MAP_BITS(eeprom.data.port_dir, zs8_port_config_fp, 0, 0);
	MAP_BITS(eeprom.data.port_dir, zs8_port_config_fp, 1, 1);
	MAP_BITS(eeprom.data.port_dir, zs8_port_config_fp, 2, 2);
	MAP_BITS(eeprom.data.port_dir, zs8_port_config_zs, 3, 6);
	MAP_BITS(eeprom.data.port_dir, zs8_port_config_zs, 4, 5);
	MAP_BITS(eeprom.data.port_dir, zs8_port_config_zs, 5, 4);
	MAP_BITS(eeprom.data.port_dir, zs8_port_config_zs, 6, 2);
	MAP_BITS(eeprom.data.port_dir, zs8_port_config_zs, 7, 1);
	MAP_BITS(eeprom.data.port_dir, zs8_port_config_zs, 8, 0);

	// Initial key state
	port_di = port_pin_status();

	uint8_t pinctrl = PORT_OPC_TOTEM_gc;
	uint16_t mask = 0x777;
	
	if (eeprom.data.ln_gpio_config&(1<<LN_GPIO_CONFIG_LNRX_PA5))
	{
		// Mask PA4-7
		mask = 0x707;
	}
	
	if (eeprom.data.port_config&(1<<PORT_MODE_PULLUP_ENABLE))
	{
		pinctrl = PORT_OPC_PULLUP_gc;
	}
	
	PORTCFG.MPCMASK = (mask&0xFF);
	PORTA.PIN0CTRL = pinctrl;
	
	PORTCFG.MPCMASK = (((mask)>>8)&0xFF);
	PORTB.PIN0CTRL = pinctrl;
	
	// Use TCD2 CCD (PWM) as trigger for ADC
	EVSYS.CH3MUX = EVSYS_CHMUX_TCD0_CCD_gc;
	
	//Load ADC calibration from production signature
	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	ADCA.CALL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	ADCA.CALH = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;

	ADCA.PRESCALER  = ADC_PRESCALER_DIV512_gc; 	// 62.5 kSPS: 32 MHz / 512
	ADCA.CTRLB      = ADC_RESOLUTION_LEFT12BIT_gc | ADC_CONMODE_bm;     // signed mode, 12 bit left adjusted
	ADCA.REFCTRL    = ADC_REFSEL_INT1V_gc;		// REF= 1V
	ADCA.EVCTRL     = ADC_SWEEP_0123_gc | ADC_EVACT_SWEEP_gc | ADC_EVSEL_3456_gc;     // Sweep all channels, event channel 3
	
	// Configure all channels as single-ended
	// and initialize the input multiplexer
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN8_gc;
	ADCA.CH0.CTRL   = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	
	ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN9_gc;
	ADCA.CH1.CTRL   = ADC_CH_INPUTMODE_SINGLEENDED_gc;

	ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN10_gc;
	ADCA.CH2.CTRL   = ADC_CH_INPUTMODE_SINGLEENDED_gc;

	ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN11_gc;
	ADCA.CH3.CTRL   = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH3.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_HI_gc;			// Enable high interrupt for ADC Channel 3
	
	ADCA.INTFLAGS = 0xF;			// Clear the interrupt flags
	ADCA.CTRLA = ADC_ENABLE_bm;		// Enable ADC
}

ISR(ADCA_CH3_vect)
{
	if (adc_phase==0)
	{
		// Disable output on PORTB
		PORTB.DIRCLR = zs8_port_config_fp;
		
		// Configure MUX to scan inputs 8-11
		ADCA.CH0.MUXCTRL = 8<<3;
		ADCA.CH1.MUXCTRL = 9<<3;
		ADCA.CH2.MUXCTRL = 10<<3;
		ADCA.CH3.MUXCTRL = 11<<3;
		
		adc_phase = 1;
		
		// Start next conversion
		ADCA.CTRLA = ADC_CH0START_bm | ADC_CH1START_bm | ADC_CH2START_bm | ADC_CH3START_bm | ADC_ENABLE_bm;
	}
	else if (adc_phase==1)
	{
		// Disable output on all PORTA but PA3 (CommRx)
		PORTA.DIRCLR = zs8_port_config_zs;
		// Enable output on PORTB
		PORTB.DIRSET = zs8_port_config_fp;
		
		// Configure MUX to scan inputs 0-3
		ADCA.CH0.MUXCTRL = 0<<3;
		ADCA.CH1.MUXCTRL = 1<<3;
		ADCA.CH2.MUXCTRL = 2<<3;
		ADCA.CH3.MUXCTRL = 3<<3;
		
		// Store results
		port_value_act[8] = (ADCA.CH0RES>>8);
		port_value_act[9] = (ADCA.CH1RES>>8);
		port_value_act[10] = (ADCA.CH2RES>>8);
		port_value_act[11] = (ADCA.CH3RES>>8);
		
		adc_phase = 2;
		
		// Start next conversion
		ADCA.CTRLA = ADC_CH0START_bm | ADC_CH1START_bm | ADC_CH2START_bm | ADC_CH3START_bm | ADC_ENABLE_bm;
	}
	else if (adc_phase==2)
	{
		// Configure MUX to scan inputs 4-7
		ADCA.CH0.MUXCTRL = 4<<3;
		ADCA.CH1.MUXCTRL = 5<<3;
		ADCA.CH2.MUXCTRL = 6<<3;
		ADCA.CH3.MUXCTRL = 7<<3;
		
		// Store results
		port_value_act[0] = (ADCA.CH0RES>>8);
		port_value_act[1] = (ADCA.CH1RES>>8);
		port_value_act[2] = (ADCA.CH2RES>>8);
		port_value_act[3] = (ADCA.CH3RES>>8);
		
		adc_phase = 3;
		
		// Start next conversion
		ADCA.CTRLA = ADC_CH0START_bm | ADC_CH1START_bm | ADC_CH2START_bm | ADC_CH3START_bm | ADC_ENABLE_bm;
	}
	else
	{
		// Enable output on PORTA
		PORTA.DIRSET = zs8_port_config_zs;
		
		// Configure MUX to scan inputs 8-11
		ADCA.CH0.MUXCTRL = 8<<3;
		ADCA.CH1.MUXCTRL = 9<<3;
		ADCA.CH2.MUXCTRL = 10<<3;
		ADCA.CH3.MUXCTRL = 11<<3;
		
		// Store results
		port_value_act[4] = (ADCA.CH0RES>>8);
		port_value_act[5] = (ADCA.CH1RES>>8);
		port_value_act[6] = (ADCA.CH2RES>>8);
		port_value_act[7] = (ADCA.CH3RES>>8);
		
		adc_phase = 0;		
	}
}


void port_update_mapping(void)
{
	port[1][0] = pwm_port[0].pwm_current;
	port[1][1] = pwm_port[1].pwm_current;
	port[1][2] = pwm_port[2].pwm_current;
	port[0][6] = pwm_port[3].pwm_current;
	port[0][5] = pwm_port[4].pwm_current;
	port[0][4] = pwm_port[5].pwm_current;
	port[0][2] = pwm_port[6].pwm_current;
	port[0][1] = pwm_port[7].pwm_current;
	port[0][0] = pwm_port[8].pwm_current;
}
