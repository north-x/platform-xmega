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
//&sbk_process,
#endif

/*
 *	SV Configuration Table
 *
 *	List of all configuration variables defined by this module
 *
 */
#ifdef SV_CFG
SV(351, "SBK In Track", sbk_in_track, sbk_update_track)
SV(352, "SBK Out Track", sbk_out_track, sbk_update_track)
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
#ifndef sbk_H_
#define sbk_H_

void sbk_init(void);
void sbk_update_track(void);
void sbk_in_abort(void);
void sbk_out_abort(void);

extern volatile uint8_t sbk_mode;
extern volatile uint8_t sbk_in_track;
extern volatile uint8_t sbk_out_track;

#define SBK_MODE_NORMAL	0
#define SBK_MODE_FSS	1

#define RELAIS_G3_OUT	2
#define RELAIS_G3_IN	3
#define RELAIS_G2_OUT	4
#define RELAIS_G2_IN	5
#define RELAIS_G1_IN	6
#define RELAIS_G1_OUT	7
#define RELAIS_G4_OUT	8
#define RELAIS_G4_IN	9
#define RELAIS_FSS		11
#define RELAIS_G6_OUT	12
#define RELAIS_G6_IN	13
#define RELAIS_G5_IN	14
#define RELAIS_G5_OUT	15

#endif /* sbk_H_ */
#endif
