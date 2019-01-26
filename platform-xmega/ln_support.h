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
&ln_process,
#endif

/*
 *	SV Configuration Table
 *
 *	List of all configuration variables defined by this module
 *
 */
#ifdef SV_CFG
SV(153, "LN Threshold Voltage x10", eeprom.ln_threshold, ln_update_threshold)
SV(154, "LN Ack Count", eeprom.ln_gpio_ack_count, 0)
SV(155, "LN GPIO Configuration", eeprom.ln_gpio_config, 0)
SV(156, "LN GPIO Direction L", eeprom.ln_gpio_dir[0], 0)
SV(157, "LN GPIO Direction H", eeprom.ln_gpio_dir[1], 0)
SV(158, "LN GPIO Status L", ln_gpio_status[0], 0)
SV(159, "LN GPIO Status H", ln_gpio_status[1], 0)
SV(160, "LN GPIO Tx L", ln_gpio_tx[0], 0)
SV(161, "LN GPIO Tx H", ln_gpio_tx[1], 0)
SV(162, "LN GPIO 1 On Opcode 1", eeprom.ln_gpio_opcode[0][0], ln_trigger_lookup_update)
SV(163, "LN GPIO 1 On Opcode 2", eeprom.ln_gpio_opcode[0][1], ln_trigger_lookup_update)
SV(164, "LN GPIO 1 On Opcode 3", eeprom.ln_gpio_opcode[0][2], ln_trigger_lookup_update)
SV(165, "LN GPIO 1 Off Opcode 1", eeprom.ln_gpio_opcode[1][0], ln_trigger_lookup_update)
SV(166, "LN GPIO 1 Off Opcode 2", eeprom.ln_gpio_opcode[1][1], ln_trigger_lookup_update)
SV(167, "LN GPIO 1 Off Opcode 3", eeprom.ln_gpio_opcode[1][2], ln_trigger_lookup_update)
SV(168, "LN GPIO 2 On Opcode 1", eeprom.ln_gpio_opcode[2][0], ln_trigger_lookup_update)
SV(169, "LN GPIO 2 On Opcode 2", eeprom.ln_gpio_opcode[2][1], ln_trigger_lookup_update)
SV(170, "LN GPIO 2 On Opcode 3", eeprom.ln_gpio_opcode[2][2], ln_trigger_lookup_update)
SV(171, "LN GPIO 2 Off Opcode 1", eeprom.ln_gpio_opcode[3][0], ln_trigger_lookup_update)
SV(172, "LN GPIO 2 Off Opcode 2", eeprom.ln_gpio_opcode[3][1], ln_trigger_lookup_update)
SV(173, "LN GPIO 2 Off Opcode 3", eeprom.ln_gpio_opcode[3][2], ln_trigger_lookup_update)
SV(174, "LN GPIO 3 On Opcode 1", eeprom.ln_gpio_opcode[4][0], ln_trigger_lookup_update)
SV(175, "LN GPIO 3 On Opcode 2", eeprom.ln_gpio_opcode[4][1], ln_trigger_lookup_update)
SV(176, "LN GPIO 3 On Opcode 3", eeprom.ln_gpio_opcode[4][2], ln_trigger_lookup_update)
SV(177, "LN GPIO 3 Off Opcode 1", eeprom.ln_gpio_opcode[5][0], ln_trigger_lookup_update)
SV(178, "LN GPIO 3 Off Opcode 2", eeprom.ln_gpio_opcode[5][1], ln_trigger_lookup_update)
SV(179, "LN GPIO 3 Off Opcode 3", eeprom.ln_gpio_opcode[5][2], ln_trigger_lookup_update)
SV(180, "LN GPIO 4 On Opcode 1", eeprom.ln_gpio_opcode[6][0], ln_trigger_lookup_update)
SV(181, "LN GPIO 4 On Opcode 2", eeprom.ln_gpio_opcode[6][1], ln_trigger_lookup_update)
SV(182, "LN GPIO 4 On Opcode 3", eeprom.ln_gpio_opcode[6][2], ln_trigger_lookup_update)
SV(183, "LN GPIO 4 Off Opcode 1", eeprom.ln_gpio_opcode[7][0], ln_trigger_lookup_update)
SV(184, "LN GPIO 4 Off Opcode 2", eeprom.ln_gpio_opcode[7][1], ln_trigger_lookup_update)
SV(185, "LN GPIO 4 Off Opcode 3", eeprom.ln_gpio_opcode[7][2], ln_trigger_lookup_update)
SV(186, "LN GPIO 5 On Opcode 1", eeprom.ln_gpio_opcode[8][0], ln_trigger_lookup_update)
SV(187, "LN GPIO 5 On Opcode 2", eeprom.ln_gpio_opcode[8][1], ln_trigger_lookup_update)
SV(188, "LN GPIO 5 On Opcode 3", eeprom.ln_gpio_opcode[8][2], ln_trigger_lookup_update)
SV(189, "LN GPIO 5 Off Opcode 1", eeprom.ln_gpio_opcode[9][0], ln_trigger_lookup_update)
SV(190, "LN GPIO 5 Off Opcode 2", eeprom.ln_gpio_opcode[9][1], ln_trigger_lookup_update)
SV(191, "LN GPIO 5 Off Opcode 3", eeprom.ln_gpio_opcode[9][2], ln_trigger_lookup_update)
SV(192, "LN GPIO 6 On Opcode 1", eeprom.ln_gpio_opcode[10][0], ln_trigger_lookup_update)
SV(193, "LN GPIO 6 On Opcode 2", eeprom.ln_gpio_opcode[10][1], ln_trigger_lookup_update)
SV(194, "LN GPIO 6 On Opcode 3", eeprom.ln_gpio_opcode[10][2], ln_trigger_lookup_update)
SV(195, "LN GPIO 6 Off Opcode 1", eeprom.ln_gpio_opcode[11][0], ln_trigger_lookup_update)
SV(196, "LN GPIO 6 Off Opcode 2", eeprom.ln_gpio_opcode[11][1], ln_trigger_lookup_update)
SV(197, "LN GPIO 6 Off Opcode 3", eeprom.ln_gpio_opcode[11][2], ln_trigger_lookup_update)
SV(198, "LN GPIO 7 On Opcode 1", eeprom.ln_gpio_opcode[12][0], ln_trigger_lookup_update)
SV(199, "LN GPIO 7 On Opcode 2", eeprom.ln_gpio_opcode[12][1], ln_trigger_lookup_update)
SV(200, "LN GPIO 7 On Opcode 3", eeprom.ln_gpio_opcode[12][2], ln_trigger_lookup_update)
SV(201, "LN GPIO 7 Off Opcode 1", eeprom.ln_gpio_opcode[13][0], ln_trigger_lookup_update)
SV(202, "LN GPIO 7 Off Opcode 2", eeprom.ln_gpio_opcode[13][1], ln_trigger_lookup_update)
SV(203, "LN GPIO 7 Off Opcode 3", eeprom.ln_gpio_opcode[13][2], ln_trigger_lookup_update)
SV(204, "LN GPIO 8 On Opcode 1", eeprom.ln_gpio_opcode[14][0], ln_trigger_lookup_update)
SV(205, "LN GPIO 8 On Opcode 2", eeprom.ln_gpio_opcode[14][1], ln_trigger_lookup_update)
SV(206, "LN GPIO 8 On Opcode 3", eeprom.ln_gpio_opcode[14][2], ln_trigger_lookup_update)
SV(207, "LN GPIO 8 Off Opcode 1", eeprom.ln_gpio_opcode[15][0], ln_trigger_lookup_update)
SV(208, "LN GPIO 8 Off Opcode 2", eeprom.ln_gpio_opcode[15][1], ln_trigger_lookup_update)
SV(209, "LN GPIO 8 Off Opcode 3", eeprom.ln_gpio_opcode[15][2], ln_trigger_lookup_update)
SV(210, "LN GPIO 9 On Opcode 1", eeprom.ln_gpio_opcode[16][0], ln_trigger_lookup_update)
SV(211, "LN GPIO 9 On Opcode 2", eeprom.ln_gpio_opcode[16][1], ln_trigger_lookup_update)
SV(212, "LN GPIO 9 On Opcode 3", eeprom.ln_gpio_opcode[16][2], ln_trigger_lookup_update)
SV(213, "LN GPIO 9 Off Opcode 1", eeprom.ln_gpio_opcode[17][0], ln_trigger_lookup_update)
SV(214, "LN GPIO 9 Off Opcode 2", eeprom.ln_gpio_opcode[17][1], ln_trigger_lookup_update)
SV(215, "LN GPIO 9 Off Opcode 3", eeprom.ln_gpio_opcode[17][2], ln_trigger_lookup_update)
SV(216, "LN GPIO 10 On Opcode 1", eeprom.ln_gpio_opcode[18][0], ln_trigger_lookup_update)
SV(217, "LN GPIO 10 On Opcode 2", eeprom.ln_gpio_opcode[18][1], ln_trigger_lookup_update)
SV(218, "LN GPIO 10 On Opcode 3", eeprom.ln_gpio_opcode[18][2], ln_trigger_lookup_update)
SV(219, "LN GPIO 10 Off Opcode 1", eeprom.ln_gpio_opcode[19][0], ln_trigger_lookup_update)
SV(220, "LN GPIO 10 Off Opcode 2", eeprom.ln_gpio_opcode[19][1], ln_trigger_lookup_update)
SV(221, "LN GPIO 10 Off Opcode 3", eeprom.ln_gpio_opcode[19][2], ln_trigger_lookup_update)
SV(222, "LN GPIO 11 On Opcode 1", eeprom.ln_gpio_opcode[20][0], ln_trigger_lookup_update)
SV(223, "LN GPIO 11 On Opcode 2", eeprom.ln_gpio_opcode[20][1], ln_trigger_lookup_update)
SV(224, "LN GPIO 11 On Opcode 3", eeprom.ln_gpio_opcode[20][2], ln_trigger_lookup_update)
SV(225, "LN GPIO 11 Off Opcode 1", eeprom.ln_gpio_opcode[21][0], ln_trigger_lookup_update)
SV(226, "LN GPIO 11 Off Opcode 2", eeprom.ln_gpio_opcode[21][1], ln_trigger_lookup_update)
SV(227, "LN GPIO 11 Off Opcode 3", eeprom.ln_gpio_opcode[21][2], ln_trigger_lookup_update)
SV(228, "LN GPIO 12 On Opcode 1", eeprom.ln_gpio_opcode[22][0], ln_trigger_lookup_update)
SV(229, "LN GPIO 12 On Opcode 2", eeprom.ln_gpio_opcode[22][1], ln_trigger_lookup_update)
SV(230, "LN GPIO 12 On Opcode 3", eeprom.ln_gpio_opcode[22][2], ln_trigger_lookup_update)
SV(231, "LN GPIO 12 Off Opcode 1", eeprom.ln_gpio_opcode[23][0], ln_trigger_lookup_update)
SV(232, "LN GPIO 12 Off Opcode 2", eeprom.ln_gpio_opcode[23][1], ln_trigger_lookup_update)
SV(233, "LN GPIO 12 Off Opcode 3", eeprom.ln_gpio_opcode[23][2], ln_trigger_lookup_update)
SV(234, "LN GPIO 13 On Opcode 1", eeprom.ln_gpio_opcode[24][0], ln_trigger_lookup_update)
SV(235, "LN GPIO 13 On Opcode 2", eeprom.ln_gpio_opcode[24][1], ln_trigger_lookup_update)
SV(236, "LN GPIO 13 On Opcode 3", eeprom.ln_gpio_opcode[24][2], ln_trigger_lookup_update)
SV(237, "LN GPIO 13 Off Opcode 1", eeprom.ln_gpio_opcode[25][0], ln_trigger_lookup_update)
SV(238, "LN GPIO 13 Off Opcode 2", eeprom.ln_gpio_opcode[25][1], ln_trigger_lookup_update)
SV(239, "LN GPIO 13 Off Opcode 3", eeprom.ln_gpio_opcode[25][2], ln_trigger_lookup_update)
SV(240, "LN GPIO 14 On Opcode 1", eeprom.ln_gpio_opcode[26][0], ln_trigger_lookup_update)
SV(241, "LN GPIO 14 On Opcode 2", eeprom.ln_gpio_opcode[26][1], ln_trigger_lookup_update)
SV(242, "LN GPIO 14 On Opcode 3", eeprom.ln_gpio_opcode[26][2], ln_trigger_lookup_update)
SV(243, "LN GPIO 14 Off Opcode 1", eeprom.ln_gpio_opcode[27][0], ln_trigger_lookup_update)
SV(244, "LN GPIO 14 Off Opcode 2", eeprom.ln_gpio_opcode[27][1], ln_trigger_lookup_update)
SV(245, "LN GPIO 14 Off Opcode 3", eeprom.ln_gpio_opcode[27][2], ln_trigger_lookup_update)
SV(246, "LN GPIO 15 On Opcode 1", eeprom.ln_gpio_opcode[28][0], ln_trigger_lookup_update)
SV(247, "LN GPIO 15 On Opcode 2", eeprom.ln_gpio_opcode[28][1], ln_trigger_lookup_update)
SV(248, "LN GPIO 15 On Opcode 3", eeprom.ln_gpio_opcode[28][2], ln_trigger_lookup_update)
SV(249, "LN GPIO 15 Off Opcode 1", eeprom.ln_gpio_opcode[29][0], ln_trigger_lookup_update)
SV(250, "LN GPIO 15 Off Opcode 2", eeprom.ln_gpio_opcode[29][1], ln_trigger_lookup_update)
SV(251, "LN GPIO 15 Off Opcode 3", eeprom.ln_gpio_opcode[29][2], ln_trigger_lookup_update)
SV(252, "LN GPIO 16 On Opcode 1", eeprom.ln_gpio_opcode[30][0], ln_trigger_lookup_update)
SV(253, "LN GPIO 16 On Opcode 2", eeprom.ln_gpio_opcode[30][1], ln_trigger_lookup_update)
SV(254, "LN GPIO 16 On Opcode 3", eeprom.ln_gpio_opcode[30][2], ln_trigger_lookup_update)
SV(255, "LN GPIO 16 Off Opcode 1", eeprom.ln_gpio_opcode[31][0], ln_trigger_lookup_update)
SV(256, "LN GPIO 16 Off Opcode 2", eeprom.ln_gpio_opcode[31][1], ln_trigger_lookup_update)
SV(257, "LN GPIO 16 Off Opcode 3", eeprom.ln_gpio_opcode[31][2], ln_trigger_lookup_update)
SV(258, "LN Watchdog Counter", ln_wdt_counter, 0)
SV(259, "LN Rx Error Counter", LnBuffer.Stats.RxErrors, 0)
SV(260, "LN Tx Error Counter", LnBuffer.Stats.TxError, 0)
SV(261, "LN Collision Counter", LnBuffer.Stats.Collisions, 0)
SV_LSB(262, "LN Rx Packets Counter L", LnBuffer.Stats.RxPackets, 0)
SV_MSB(263, "LN Rx Packets Counter H", LnBuffer.Stats.RxPackets, 0)
SV_LSB(264, "LN Tx Packets Counter L", LnBuffer.Stats.TxPackets, 0)
SV_MSB(265, "LN Rx Packets Counter H", LnBuffer.Stats.TxPackets, 0)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
uint8_t ln_threshold;
uint8_t ln_gpio_config;
uint8_t ln_gpio_ack_count;
uint8_t ln_gpio_dir[2];
uint8_t ln_gpio_opcode[32][3];
#endif

/*
 *	EEPROM Status Variable Definition
 */
#ifdef EEPROM_STATUS_CFG
uint8_t ln_gpio_status[2];
#endif

/*
 *	EEPROM Configuration Variable Default Configuration
 */
#ifdef EEPROM_DEFAULT
.ln_threshold = 20,
.ln_gpio_config = (1<<LN_GPIO_CONFIG_GPON),
.ln_gpio_ack_count = 0,
.ln_gpio_dir = { 0, 0},
.ln_gpio_opcode = {
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0}
},
#endif

/*
 *	EEPROM Status Variable Default Configuration
 */
#ifdef EEPROM_STATUS_DEFAULT
.ln_gpio_status = { 0, 0 },
#endif

/*
 * LN Receive Callback Definition
 *
 * Function to be called when a valid packet was received
 */
#ifdef LN_RX_CALLBACK
//LN_RX_CALLBACK(ln_throttle_process)
#endif

/*
 * SV CMD Callback Definition
 *
 * Function to be called when SV#5 is written
 */
#ifdef SV_CMD_CALLBACK
SV_CMD_CALLBACK(lns_sv_cmd_callback)
#endif

#else
/************************************************************************/
/* Module Header File                                                   */
/************************************************************************/
#ifndef LN_SUPPORT_H_
#define LN_SUPPORT_H_

#include "loconet.h"
#include "ln_buf.h"

#define LN_GPIO_CH_COUNT	16
#define LN_GPIO_BW		((LN_GPIO_CH_COUNT-1)/8L)+1

#define LN_LOOKUP_ENTRIES	32
#define LN_LOOKUP_MASK		(LN_LOOKUP_ENTRIES-1)

#if ( LN_LOOKUP_ENTRIES & LN_LOOKUP_MASK )
#error LN lookup size is not a power of 2
#endif

#define LN_GPIO_CONFIG_GPON		0
#define LN_GPIO_CONFIG_STARTUP	1
#define LN_GPIO_CONFIG_WATCHDOG	7

extern uint8_t ln_gpio_tx[LN_GPIO_BW];
extern uint8_t ln_gpio_tx_ack[LN_GPIO_BW];
extern uint8_t ln_gpio_status[LN_GPIO_BW];
extern uint8_t ln_gpio_status_pre[LN_GPIO_BW];
extern uint8_t ln_gpio_status_ack[LN_GPIO_BW];
extern uint8_t ln_gpio_status_flag[LN_GPIO_BW];
extern uint8_t ln_wdt_counter;
extern LnBuf LnBuffer;
extern uint8_t ln_gpio_lookup[LN_LOOKUP_ENTRIES];
extern uint8_t ln_gpio_lookup_list[2*LN_GPIO_CH_COUNT];

void loconet_init(void);
uint8_t ln_create_message(uint8_t *msg);
uint8_t ln_create_message_ack(uint8_t *msg);
uint8_t ln_is_ack_message(uint8_t *msg);
void ln_gpio_process_tx(void);
void ln_gpio_process_rx(lnMsg *LnPacket, uint8_t source);
void ln_create_opcode(uint8_t *buf, uint8_t opc, uint16_t addr);
void ln_update_threshold(void);
void ln_trigger_lookup_update(void);
void ln_update_lookup(void);
uint8_t getLnMsgChecksum(lnMsg *msg);
void lns_sv_cmd_callback(uint8_t cmd);

PROCESS_NAME(ln_process);

#endif /* LN_SUPPORT_H_ */
#endif