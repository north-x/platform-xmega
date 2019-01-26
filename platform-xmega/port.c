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

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include "sys/process.h"
#include "sys/etimer.h"
#include "loconet.h"
#include "port.h"
#include "eeprom.h"
#include "ln_support.h"

uint8_t port[4][8]; // 4 ports A-D, 8 outputs each, 0=off, 63=on
t_pwm_port pwm_port[PWM_PORT_COUNT];
int8_t pwm_target[PWM_PORT_COUNT];
uint16_t pwm_update_trig;
uint16_t pwm_at_setpoint;
uint8_t pwm_target_temp;
uint8_t pwm_delta_temp;
uint16_t pwm_parameter_select;

uint16_t port_do;
uint16_t port_di;
uint16_t port_di_mapped;
uint16_t port_do_mapped;
uint16_t port_user;
uint8_t port_blink_phase;
uint8_t port_blink_count;

// Gamma correction tables
uint8_t pwm_gamma_4b6b[16] = { 0, 1, 2, 5, 8, 11, 15, 19, 24, 28, 34, 39, 44, 50, 56, 63 };
uint8_t pwm_gamma_5b6b[32] = { 0, 1, 2, 3, 4, 5, 7, 8, 9, 11, 13, 14, 16, 18, 20, 22, 24, 27, 29, 32, 34, 37, 39, 42, 45, 48, 51, 54, 58, 61, 63, 63 };
// Binary code modulation tables (4 ports, 6 timeslices)
uint8_t pwm_bitslice[6][4];
uint8_t pwm_bitslice_shadow[6][4];
volatile uint8_t pwm_bitslice_update_needed;

static struct etimer port_timer;

PROCESS(port_process, "Port IO Handling");

PROCESS_THREAD(port_process, ev, data)
{
	static uint16_t ct0, ct1;
	uint16_t i, in;
	
	PROCESS_BEGIN();
	
	port_init();
	etimer_set(&port_timer, 20E-3*CLOCK_SECOND);
	
	while(1)
	{
		PROCESS_YIELD();
		// DI (digital input) handling
		in = port_pin_status();

		i = port_di ^ ~in;			// key changed ?

		ct0 = ~( ct0 & i );			// reset or count ct0
		ct1 = ct0 ^ (ct1 & i);		// reset or count ct1
		i &= ct0 & ct1;				// count until roll over ?
		port_di ^= i;				// then toggle debounced state
		
		port_blink_count++;
		if (port_blink_count>25)
		{
			port_blink_count = 0;
			port_blink_phase++;
		}

		port_do = ln_gpio_status[0] | (ln_gpio_status[1]<<8);
		port_do_mapping();
		pwm_step();
		port_update_mapping();
		pwm_update_bitslices();


		// Overwrite status bits if dir=1 (LN output)
		ln_gpio_status[0] = (ln_gpio_status[0]&(~eeprom.ln_gpio_dir[0]))|(((uint8_t) port_di_mapped)&eeprom.ln_gpio_dir[0]);
		ln_gpio_status[1] = (ln_gpio_status[1]&(~eeprom.ln_gpio_dir[1]))|(((uint8_t) (port_di_mapped>>8))&eeprom.ln_gpio_dir[1]);
		
		etimer_reset(&port_timer);
	}
	
	PROCESS_END();
}

void port_update_configuration(void)
{	
	port_di_init();
	pwm_init();
}

void port_init(void)
{	
	port_di_init();
	pwm_init();
}

void pwm_step(void)
{
	uint8_t index;
	
	for (index=0;index<PWM_PORT_COUNT;index++)
	{
		if ((pwm_update_trig&(1<<index))!=0)
		{
			pwm_port[index].dimm_delta = eeprom.pwm_delta[index];
			pwm_port[index].dimm_target = pwm_target[index];
			if (!(eeprom.port_config&(1<<PORT_MODE_PWM_UPDATE_CONT))) {
				pwm_update_trig &= ~(1<<index);			
			}
		}
				
		if (pwm_port[index].dimm_current==pwm_port[index].dimm_target)
		{
			pwm_at_setpoint |= (1<<index);
			continue;
		}
		else
		{
			pwm_at_setpoint &= ~(1<<index);
		}
		
		if (pwm_port[index].dimm_current>(pwm_port[index].dimm_target+pwm_port[index].dimm_delta))
		{
			pwm_port[index].dimm_current -= pwm_port[index].dimm_delta;
		}
		else if (pwm_port[index].dimm_current<(pwm_port[index].dimm_target-pwm_port[index].dimm_delta))
		{
			pwm_port[index].dimm_current += pwm_port[index].dimm_delta;
		}
		else
		{
			pwm_port[index].dimm_current = pwm_port[index].dimm_target;
		}
		
		if (pwm_port[index].dimm_current<DIMM_RANGE_MIN)
		{
			pwm_port[index].pwm_current = pwm_gamma_5b6b[DIMM_RANGE_MIN];
		}
		else if (pwm_port[index].dimm_current>DIMM_RANGE_MAX)
		{
			pwm_port[index].pwm_current = pwm_gamma_5b6b[DIMM_RANGE_MAX];
		}
		else
		{
			pwm_port[index].pwm_current = pwm_gamma_5b6b[pwm_port[index].dimm_current];
		}
	}
}

/*
Offset	Description
0		port_do
16		port_di
32		port_do_mapped
48		port_di_mapped
64		port_do & blink
80		port_do & ~blink
96		~port_do & blink
112		~port_do & ~blink
128		port_user
144		port_blink_phase
*/
uint8_t port_map_get_bit(uint8_t index)
{
	if (index<16)
	{
		return port_do&(1<<index) ? 1 : 0;
	}
	else if (index<32)
	{
		return port_di&(1<<(index-16)) ? 1 : 0;
	}
	else if (index<48)
	{
		return port_do_mapped&(1<<(index-32)) ? 1 : 0;
	}
	else if (index<64)
	{
		return port_di_mapped&(1<<(index-48)) ? 1 : 0;
	}
	else if (index<80)
	{
		return port_do&(1<<(index-64)) ? port_blink_phase&1 : 0;
	}
	else if (index<96)
	{
		return port_do&(1<<(index-80)) ? (~port_blink_phase)&1 : 0;
	}
	else if (index<112)
	{
		return port_do&(1<<(index-96)) ? 0: port_blink_phase&1;
	}
	else if (index<128)
	{
		return port_do&(1<<(index-112)) ? 0: (~port_blink_phase)&1;
	}
	else if (index<144)
	{
		return port_user&(1<<(index-128)) ? 1 : 0;
	}
	else if (index<152)
	{
		return port_blink_phase&(1<<(index-136)) ? 1 : 0;
	}
	
	return 0;
}

void port_do_mapping(void)
{
	uint8_t index;
	uint8_t lut_bit;
	uint16_t status_mapped = 0;
		
	// Determine new input or output state
	for (index=0;index<16;index++)
	{
		lut_bit  = port_map_get_bit(eeprom.port_map_mux0[index])? (1<<2) : 0;
		lut_bit |= port_map_get_bit(eeprom.port_map_mux1[index])? (1<<1) : 0;
		lut_bit |= port_map_get_bit(eeprom.port_map_mux2[index])? (1<<0) : 0;
						
		status_mapped |= (eeprom.port_map_lut[index]&(1<<lut_bit))? (1<<index) : 0;
	}
	
	// Assign inputs and outputs (brightness)
	for (index=0;index<16;index++)
	{
		if ((eeprom.port_map_dir&(1<<index))==0)
		{
			if (status_mapped&(1<<index))
			{
				port_di_mapped |= (1<<index);
			}
			else
			{
				port_di_mapped &= ~(1<<index);
			}
		}
		else
		{
			if (status_mapped&(1<<index))
			{
				port_do_mapped |= (1<<index);
				if (!(eeprom.port_pwm_enable&(1<<index)))
				{
					pwm_port[index].dimm_target = eeprom.port_brightness_on[index];
				}
			}
			else
			{
				port_do_mapped &= ~(1<<index);
			
				if (!(eeprom.port_pwm_enable&(1<<index)))
				{
					pwm_port[index].dimm_target = eeprom.port_brightness_off[index];
				}
			}
		}
	}	
}

void pwm_init(void)
{
	uint8_t index;
	
	// Load settings from EEPROM
	for (index=0;index<PWM_PORT_COUNT;index++)
	{
		pwm_port[index].dimm_delta = eeprom.pwm_delta[index];
	}

	pwm_update_bitslices();
	
	PORTCFG.VPCTRLA = PORTCFG_VP02MAP_PORTA_gc | PORTCFG_VP13MAP_PORTB_gc;
	PORTCFG.VPCTRLB = PORTCFG_VP02MAP_PORTC_gc | PORTCFG_VP13MAP_PORTD_gc;
	
	EVSYS.CH2MUX = EVSYS_CHMUX_OFF_gc;
	
	// Enable Timer Type 2
	TCD2.CTRLE = TC2_BYTEM_SPLITMODE_gc;
	TCD2.LPER = PWM_STEPS-1;
	TCD2.HPER = PWM_STEPS-1;
	TCD2.CTRLB = 0; // Disable compare channels
	TCD2.CTRLA = TC2_CLKSEL_EVCH2_gc;
	
	TCD2.LCNT = 1;
	TCD2.HCNT = 0;
	
	TCD2.LCMPA = PWM_STEPS-2;
	TCD2.LCMPB = PWM_STEPS-2-4;
	TCD2.LCMPC = PWM_STEPS-2-4-8;
	TCD2.LCMPD = PWM_STEPS-2-4-8-16;

	// Clear all interrupt flags
	TCD2.INTFLAGS = 0xFF;
	
	// Enable interrupt on TCD2
	TCD2.INTCTRLA = TC2_HUNFINTLVL_LO_gc|TC2_LUNFINTLVL_LO_gc;
	TCD2.INTCTRLB = TC2_LCMPAINTLVL_LO_gc|TC2_LCMPBINTLVL_LO_gc|TC2_LCMPCINTLVL_LO_gc|TC2_LCMPDINTLVL_LO_gc;
	
	// Event CH2: 32 MHz / 2048 = 15625 Hz clock = 244 Hz PWM frequency
	EVSYS.CH2MUX = EVSYS_CHMUX_PRESCALER_2048_gc;
}

void pwm_target_parameter_update(void)
{
	uint8_t index;
	
	for (index=0;index<PWM_PORT_COUNT;index++)
	{
		if (pwm_parameter_select&(1<<index))
		{
			pwm_port[index].dimm_target = pwm_target_temp;
		}
	}
}

void pwm_delta_parameter_update(void)
{
	uint8_t index;

	for (index=0;index<PWM_PORT_COUNT;index++)
	{
		if (pwm_parameter_select&(1<<index))
		{
			pwm_port[index].dimm_delta = pwm_delta_temp;
		}
	}
}

void pwm_update_bitslices(void)
{
	pwm_bitslice_update_needed = 0;
	// t <- 5 bit brightness
	// r <- bitslice, that needs to be stored after each for loop
	uint8_t *t; //[8];
	uint8_t r;
	uint8_t *p_bitslice;

	// PortA
	t = &(port[0][0]);
	p_bitslice = &(pwm_bitslice_shadow[0][0]);
	for( uint8_t bitmask = 0b00000001 ; bitmask < 0b01000000 ; bitmask <<= 1 )
	{
		r =  (t[0] & bitmask) ? 0b00000001 : 0 ;
		r |= (t[1] & bitmask) ? 0b00000010 : 0 ;
		r |= (t[2] & bitmask) ? 0b00000100 : 0 ;
		r |= (t[3] & bitmask) ? 0b00001000 : 0 ;
		r |= (t[4] & bitmask) ? 0b00010000 : 0 ;
		r |= (t[5] & bitmask) ? 0b00100000 : 0 ;
		r |= (t[6] & bitmask) ? 0b01000000 : 0 ;
		r |= (t[7] & bitmask) ? 0b10000000 : 0 ;
		*p_bitslice = r ;
		p_bitslice += 4 ;
	}
	
	// PortB
	t = &(port[1][0]);
	p_bitslice = &(pwm_bitslice_shadow[0][1]);
	for( uint8_t bitmask = 0b00000001 ; bitmask < 0b01000000 ; bitmask <<= 1 )
	{
		r =  (t[0] & bitmask) ? 0b00000001 : 0 ;
		r |= (t[1] & bitmask) ? 0b00000010 : 0 ;
		r |= (t[2] & bitmask) ? 0b00000100 : 0 ;
		r |= (t[3] & bitmask) ? 0b00001000 : 0 ;
		r |= (t[4] & bitmask) ? 0b00010000 : 0 ;
		r |= (t[5] & bitmask) ? 0b00100000 : 0 ;
		r |= (t[6] & bitmask) ? 0b01000000 : 0 ;
		r |= (t[7] & bitmask) ? 0b10000000 : 0 ;
		*p_bitslice = r ;
		p_bitslice += 4 ;
	}
	
	// PortC
	t = &(port[2][0]);
	p_bitslice = &(pwm_bitslice_shadow[0][2]);
	for( uint8_t bitmask = 0b00000001 ; bitmask < 0b01000000 ; bitmask <<= 1 )
	{
		r =  (t[0] & bitmask) ? 0b00000001 : 0 ;
		r |= (t[1] & bitmask) ? 0b00000010 : 0 ;
		r |= (t[2] & bitmask) ? 0b00000100 : 0 ;
		r |= (t[3] & bitmask) ? 0b00001000 : 0 ;
		r |= (t[4] & bitmask) ? 0b00010000 : 0 ;
		r |= (t[5] & bitmask) ? 0b00100000 : 0 ;
		r |= (t[6] & bitmask) ? 0b01000000 : 0 ;
		r |= (t[7] & bitmask) ? 0b10000000 : 0 ;
		*p_bitslice = r ;
		p_bitslice += 4 ;
	}
	
	// PortD
	t = &(port[3][0]);
	p_bitslice = &(pwm_bitslice_shadow[0][3]);
	for( uint8_t bitmask = 0b00000001 ; bitmask < 0b01000000 ; bitmask <<= 1 )
	{
		r =  (t[0] & bitmask) ? 0b00000001 : 0 ;
		r |= (t[1] & bitmask) ? 0b00000010 : 0 ;
		r |= (t[2] & bitmask) ? 0b00000100 : 0 ;
		r |= (t[3] & bitmask) ? 0b00001000 : 0 ;
		r |= (t[4] & bitmask) ? 0b00010000 : 0 ;
		r |= (t[5] & bitmask) ? 0b00100000 : 0 ;
		r |= (t[6] & bitmask) ? 0b01000000 : 0 ;
		r |= (t[7] & bitmask) ? 0b10000000 : 0 ;
		*p_bitslice = r ;
		p_bitslice += 4 ;
	}
	
	pwm_bitslice_update_needed = 1;
}

#if 0
ISR(TCD0_CCA_vect)
{
	static uint8_t pwm_bitpos = 0;
	
	pwm_bitpos++;
	if (pwm_bitpos>5)
	{
		pwm_bitpos = 0;
		TCD0.CCA = 1;
	}
	else
	{
		TCD0.CCA <<= 1;
	}
	
	VPORT0.OUT = pwm_bitslice[pwm_bitpos][0];
	VPORT1.OUT = pwm_bitslice[pwm_bitpos][1];
	VPORT2.OUT = pwm_bitslice[pwm_bitpos][2];
	VPORT3.OUT = pwm_bitslice[pwm_bitpos][3];
	
}
#endif

ISR(TCD2_HUNF_vect)
{
#if 1
	//__asm__ __volatile ("push r24" "\n\t");
	VPORT0.OUT = pwm_bitslice[0][0];
	VPORT1.OUT = pwm_bitslice[0][1];
	VPORT2.OUT = pwm_bitslice[0][2];
	VPORT3.OUT = pwm_bitslice[0][3];
	//__asm__ __volatile ("pop r24" "\n\t");
	//__asm__ __volatile ("reti" "\n\t");
#else	
	__asm__ __volatile (
	"push r24" "\n\t"
	"lds r24, %[inreg1]" "\n\t"
	"out %[outreg1], r24" "\n\t"
	"lds r24, %[inreg2]" "\n\t"
	"out %[outreg2], r24" "\n\t"
	"lds r24, %[inreg3]" "\n\t"
	"out %[outreg3], r24" "\n\t"
	"lds r24, %[inreg4]" "\n\t"
	"out %[outreg4], r24" "\n\t"
	"pop r24" "\n\t"
	"reti" "\n\t"
	: /* output operands */
	
	: /* input operands */
	[inreg1]	""	(&pwm_bitslice[0][0]),
	[inreg2]	""	(&pwm_bitslice[0][1]),
	[inreg3]	""	(&pwm_bitslice[0][2]),
	[inreg4]	""	(&pwm_bitslice[0][3]),
	[outreg1]	"M" (_SFR_IO_ADDR (VPORT0.OUT)),
	[outreg2]	"M" (_SFR_IO_ADDR (VPORT1.OUT)),
	[outreg3]	"M" (_SFR_IO_ADDR (VPORT2.OUT)),
	[outreg4]	"M" (_SFR_IO_ADDR (VPORT3.OUT))
	
	/* no clobbers */
	);
#endif
}

ISR(TCD2_LUNF_vect)
{
	VPORT0.OUT = pwm_bitslice[1][0];
	VPORT1.OUT = pwm_bitslice[1][1];
	VPORT2.OUT = pwm_bitslice[1][2];
	VPORT3.OUT = pwm_bitslice[1][3];
}

ISR(TCD2_LCMPA_vect)
{
	VPORT0.OUT = pwm_bitslice[2][0];
	VPORT1.OUT = pwm_bitslice[2][1];
	VPORT2.OUT = pwm_bitslice[2][2];
	VPORT3.OUT = pwm_bitslice[2][3];	
}

ISR(TCD2_LCMPB_vect)
{
	VPORT0.OUT = pwm_bitslice[3][0];
	VPORT1.OUT = pwm_bitslice[3][1];
	VPORT2.OUT = pwm_bitslice[3][2];
	VPORT3.OUT = pwm_bitslice[3][3];
}

ISR(TCD2_LCMPC_vect)
{
	VPORT0.OUT = pwm_bitslice[4][0];
	VPORT1.OUT = pwm_bitslice[4][1];
	VPORT2.OUT = pwm_bitslice[4][2];
	VPORT3.OUT = pwm_bitslice[4][3];
}

ISR(TCD2_LCMPD_vect)
{
	VPORT0.OUT = pwm_bitslice[5][0];
	VPORT1.OUT = pwm_bitslice[5][1];
	VPORT2.OUT = pwm_bitslice[5][2];
	VPORT3.OUT = pwm_bitslice[5][3];
	
	if (pwm_bitslice_update_needed)
	{
		memcpy(pwm_bitslice, pwm_bitslice_shadow, sizeof(pwm_bitslice));
		pwm_bitslice_update_needed = 0;
	}
}

void port_ln_callback(lnMsg *LnPacket)
{
	if (LnPacket->data[0] == 0x8F)
	{
		port_blink_count = 0;
		port_blink_phase = 0;
	}
}