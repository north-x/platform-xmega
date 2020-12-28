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

/************************************************************************/
/* Module Configuration                                                 */
/************************************************************************/
#ifdef CONFIGURATION

/*
 *	Autostart List
 *
 *	List of all processes that are started automatically on startup
 *
 */
#ifdef AUTOSTART_CFG
&zs8_process,
#endif

/*
 *	SV Configuration Table
 *
 *	List of all configuration variables defined by this module
 *
 */
#ifdef SV_CFG
SV(367, "Z Section 1 Mux", zs_state[0], 0)
SV(368, "Z Section 2 Mux", zs_state[1], 0)
SV(369, "Z Section 3 Mux", zs_state[2], 0)
SV(370, "Z Section 4 Mux", zs_state[3], 0)
SV(371, "Z Section 5 Mux", zs_state[4], 0)
SV(372, "Z Section 6 Mux", zs_state[5], 0)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
#endif

/*
 *	EEPROM Status Variable Definition
 */
#ifdef EEPROM_STATUS_CFG
#endif

/*
 *	EEPROM Configuration Variable Default Configuration
 */
#ifdef EEPROM_DEFAULT
#endif

/*
 *	EEPROM Status Variable Default Configuration
 */
#ifdef EEPROM_STATUS_DEFAULT
#endif

#else
/************************************************************************/
/* Module Header File                                                   */
/************************************************************************/
#ifndef zs8_H_
#define zs8_H_

PROCESS_NAME(zs8_process);

void zs8_init(void);
void zs_update(void);
void zs_process_port(void);
extern uint8_t zs_state[6];

#define ZS_PORT_ZS1_bm	(1<<9)
#define ZS_PORT_ZS2_bm	(1<<10)
#define ZS_PORT_ZS3_bm	(1<<11)
#define ZS_PORT_ZS4_bm	(1<<12)
#define ZS_PORT_ZS5_bm	(1<<13)
#define ZS_PORT_ZS6_bm	(1<<14)

#define ZS_PORT_ADDR0_bm	(1<<0)
#define ZS_PORT_ADDR1_bm	(1<<1)
#define ZS_PORT_ADDR2_bm	(1<<2)

#define ZS_PORT_MASK	0x7E07

#define PORT_MODE_ZS8_INTERFACE_ENABLE 2

// Outputs of IC4 (last shift register)
#define REL1	(1<<4)
#define REL2	(1<<6)
#define REL3	(1<<7)
#define REL4	(1<<5)
#define REL5	(1<<3)
#define REL6	(1<<1)
#define REL7	(1<<2)
#define REL8	(1<<0)

// Outputs of IC3 (second shift register)
#define REL13	(1<<7)
#define REL14	(1<<5)
#define REL15	(1<<6)
#define REL16	(1<<4)
#define REL17	(1<<0)
#define REL18	(1<<2)
#define REL19	(1<<3)
#define REL20	(1<<1)

// Outputs of IC2 (first shift register)
#define REL9	(1<<3)
#define REL10	(1<<1)
#define REL11	(1<<0)
#define REL12	(1<<2)
#define REL21	(1<<7)
#define REL22	(1<<5)
#define REL23	(1<<6)
#define REL24	(1<<4)


#endif /* zs8_H_ */
#endif
