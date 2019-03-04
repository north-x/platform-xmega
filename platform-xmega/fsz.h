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
//&fsz_process,
#endif

/*
 *	SV Configuration Table
 *
 *	List of all configuration variables defined by this module
 *
 */
#ifdef SV_CFG
SV(353, "FSZ Set Num", fsz_sv_temp, fsz_sv_helper_set)
SV(354, "FSZ Clear Num", fsz_sv_temp, fsz_sv_helper_clear)
SV_LSB(355, "FSZ1 Set LSB", fsz_register[0].set, 0)
SV_MSB(356, "FSZ1 Set MSB", fsz_register[0].set, 0)
SV_LSB(357, "FSZ1 Clear LSB", fsz_register[0].clear, 0)
SV_MSB(358, "FSZ1 Clear MSB", fsz_register[0].clear, 0)
SV_LSB(359, "FSZ1 Reg LSB", fsz_register[0].value, 0)
SV_MSB(360, "FSZ1 Reg MSB", fsz_register[0].value, 0)
SV_LSB(361, "FSZ2 Set LSB", fsz_register[1].set, 0)
SV_MSB(362, "FSZ2 Set MSB", fsz_register[1].set, 0)
SV_LSB(363, "FSZ2 Clear LSB", fsz_register[1].clear, 0)
SV_MSB(364, "FSZ2 Clear MSB", fsz_register[1].clear, 0)
SV_LSB(365, "FSZ2 Reg LSB", fsz_register[1].value, 0)
SV_MSB(366, "FSZ2 Reg MSB", fsz_register[1].value, 0)
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
#ifndef fsz_H_
#define fsz_H_

typedef struct fsz_reg {
	uint16_t value;
	uint16_t shadow;
	uint16_t set;
	uint16_t clear;
} fsz_reg_t;

extern fsz_reg_t fsz_register[2];
extern uint8_t fsz_sr_reg[4];
extern uint8_t fsz_sv_temp;

void fsz_sv_helper_set(void);
void fsz_sv_helper_clear(void);

#endif /* fsz_H_ */
#endif
