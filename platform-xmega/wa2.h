/*
 * Copyright (c) 2017, Manuel Vetterli
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
&relay_process,
&wa2_process,
&dwarf_process,
#endif

/*
 *	SV Configuration Table
 *
 *	List of all configuration variables defined by this module
 *
 */
#ifdef SV_CFG
SV(280, "Servo Multipos Opcode Select", eeprom.data.servo_multipos_opcode, 0)
SV(281, "Servo 1 Multipos Off", eeprom.data.servo_multipos[0][0], 0)
SV(282, "Servo 1 Multipos On", eeprom.data.servo_multipos[0][1], 0)
SV(283, "Servo 2 Multipos Off", eeprom.data.servo_multipos[1][0], 0)
SV(284, "Servo 2 Multipos On", eeprom.data.servo_multipos[1][1], 0)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
uint8_t servo_multipos_opcode;
uint8_t servo_multipos[2][2];
#endif

/*
 *	EEPROM Status Variable Definition
 */
#ifdef EEPROM_STATUS_CFG
uint8_t relay_request;
#endif

/*
 *	EEPROM Configuration Variable Default Configuration
 */
#ifdef EEPROM_DEFAULT
.servo_multipos_opcode = 0,
.servo_multipos = {{64, 192}, {64, 192}},
#endif

/*
 *	EEPROM Status Variable Default Configuration
 */
#ifdef EEPROM_STATUS_DEFAULT
.relay_request = 0,
#endif

/*
 * LN Receive Callback Definition
 *
 * Function to be called when a valid packet was received
 */
#ifdef LN_RX_CALLBACK
LN_RX_CALLBACK(ln_throttle_process)
#endif

/*
 * SV CMD Callback Definition
 *
 * Function to be called when SV#5 is written
 */
#ifdef SV_CMD_CALLBACK
SV_CMD_CALLBACK(ln_sv_cmd_callback)
#endif


#else
/************************************************************************/
/* Module Header File                                                   */
/************************************************************************/
#ifndef wa2_H_
#define wa2_H_


PROCESS_NAME(wa2_process);
PROCESS_NAME(relay_process);
PROCESS_NAME(dwarf_process);
void ln_throttle_process(lnMsg *LnPacket);
void ln_sv_cmd_callback(uint8_t cmd);

void relay_governor(void);

void servo_power_enable(void);
void servo_power_disable(void);

#define PU_SERVO_POWER	0
#define	PU_RELAY_1		1
#define PU_RELAY_2		2
#define PU_RELAY_RC1	3
#define PU_RELAY_RC2	4
#define PU_RELAY_RC3	5
#define PU_RELAY_RC4	6
#define PU_RELAY2_REQ	7
#define PU_DWARF1_L		8
#define PU_DWARF1_R		9
#define PU_DWARF1_T		10
#define PU_DWARF2_L		11
#define PU_DWARF2_R		12
#define PU_DWARF2_T		13


#define RELAY_CMD_LEFT1		1
#define RELAY_CMD_RIGHT1	2
#define RELAY_CMD_LEFT2		4
#define RELAY_CMD_RIGHT2	8
#define RELAY_CMD_LEFT		1
#define RELAY_CMD_RIGHT		2

#endif /* wa2_H_ */
#endif