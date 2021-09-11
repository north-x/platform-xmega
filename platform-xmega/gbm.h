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
//&gbm_process,
#endif

/*
 *	SV Configuration Table
 *
 *	List of all configuration variables defined by this module
 *
 */
#ifdef SV_CFG
SV(285, "GBM Value 1", gbm_avg[0], 0)
SV(286, "GBM Value 2", gbm_avg[1], 0)
SV(287, "GBM Value 3", gbm_avg[2], 0)
SV(288, "GBM Value 4", gbm_avg[3], 0)
SV(289, "GBM Value 5", gbm_avg[4], 0)
SV(290, "GBM Value 6", gbm_avg[5], 0)
SV(291, "GBM Value 7", gbm_avg[6], 0)
SV(292, "GBM Value 8", gbm_avg[7], 0)
SV(293, "GBM Value 9", gbm_avg[8], 0)
SV(294, "GBM Value 10", gbm_avg[9], 0)
SV(295, "GBM Value 11", gbm_avg[10], 0)
SV(296, "GBM Value 12", gbm_avg[11], 0)
SV(297, "GBM Track Select L", gbm_track_select_L, 0)
SV(298, "GBM Track Select H", gbm_track_select_H, 0)
SV(299, "GBM Threshold ON Multi", gbm_temp_multi, gbm_helper_multi_threshold_on)
SV(300, "GBM Threshold OFF Multi", gbm_temp_multi, gbm_helper_multi_threshold_off)
SV(301, "GBM Delay ON Multi", gbm_temp_multi, gbm_helper_multi_delay_on)
SV(302, "GBM Delay OFF Multi", gbm_temp_multi, gbm_helper_multi_delay_off)
SV(303, "GBM Threshold ON 1", eeprom.data.gbm_threshold_on[0], 0)
SV(304, "GBM Threshold ON 2", eeprom.data.gbm_threshold_on[1], 0)
SV(305, "GBM Threshold ON 3", eeprom.data.gbm_threshold_on[2], 0)
SV(306, "GBM Threshold ON 4", eeprom.data.gbm_threshold_on[3], 0)
SV(307, "GBM Threshold ON 5", eeprom.data.gbm_threshold_on[4], 0)
SV(308, "GBM Threshold ON 6", eeprom.data.gbm_threshold_on[5], 0)
SV(309, "GBM Threshold ON 7", eeprom.data.gbm_threshold_on[6], 0)
SV(310, "GBM Threshold ON 8", eeprom.data.gbm_threshold_on[7], 0)
SV(311, "GBM Threshold ON 9", eeprom.data.gbm_threshold_on[8], 0)
SV(312, "GBM Threshold ON 10", eeprom.data.gbm_threshold_on[9], 0)
SV(313, "GBM Threshold ON 11", eeprom.data.gbm_threshold_on[10], 0)
SV(314, "GBM Threshold ON 12", eeprom.data.gbm_threshold_on[11], 0)
SV(315, "GBM Threshold OFF 1", eeprom.data.gbm_threshold_off[0], 0)
SV(316, "GBM Threshold OFF 2", eeprom.data.gbm_threshold_off[1], 0)
SV(317, "GBM Threshold OFF 3", eeprom.data.gbm_threshold_off[2], 0)
SV(318, "GBM Threshold OFF 4", eeprom.data.gbm_threshold_off[3], 0)
SV(319, "GBM Threshold OFF 5", eeprom.data.gbm_threshold_off[4], 0)
SV(320, "GBM Threshold OFF 6", eeprom.data.gbm_threshold_off[5], 0)
SV(321, "GBM Threshold OFF 7", eeprom.data.gbm_threshold_off[6], 0)
SV(322, "GBM Threshold OFF 8", eeprom.data.gbm_threshold_off[7], 0)
SV(323, "GBM Threshold OFF 9", eeprom.data.gbm_threshold_off[8], 0)
SV(324, "GBM Threshold OFF 10", eeprom.data.gbm_threshold_off[9], 0)
SV(325, "GBM Threshold OFF 11", eeprom.data.gbm_threshold_off[10], 0)
SV(326, "GBM Threshold OFF 12", eeprom.data.gbm_threshold_off[11], 0)
SV(327, "GBM Delay ON 1", eeprom.data.gbm_delay_on[0], 0)
SV(328, "GBM Delay ON 2", eeprom.data.gbm_delay_on[1], 0)
SV(329, "GBM Delay ON 3", eeprom.data.gbm_delay_on[2], 0)
SV(330, "GBM Delay ON 4", eeprom.data.gbm_delay_on[3], 0)
SV(331, "GBM Delay ON 5", eeprom.data.gbm_delay_on[4], 0)
SV(332, "GBM Delay ON 6", eeprom.data.gbm_delay_on[5], 0)
SV(333, "GBM Delay ON 7", eeprom.data.gbm_delay_on[6], 0)
SV(334, "GBM Delay ON 8", eeprom.data.gbm_delay_on[7], 0)
SV(335, "GBM Delay ON 9", eeprom.data.gbm_delay_on[8], 0)
SV(336, "GBM Delay ON 10", eeprom.data.gbm_delay_on[9], 0)
SV(337, "GBM Delay ON 11", eeprom.data.gbm_delay_on[10], 0)
SV(338, "GBM Delay ON 12", eeprom.data.gbm_delay_on[11], 0)
SV(339, "GBM Delay OFF 1", eeprom.data.gbm_delay_off[0], 0)
SV(340, "GBM Delay OFF 2", eeprom.data.gbm_delay_off[1], 0)
SV(341, "GBM Delay OFF 3", eeprom.data.gbm_delay_off[2], 0)
SV(342, "GBM Delay OFF 4", eeprom.data.gbm_delay_off[3], 0)
SV(343, "GBM Delay OFF 5", eeprom.data.gbm_delay_off[4], 0)
SV(344, "GBM Delay OFF 6", eeprom.data.gbm_delay_off[5], 0)
SV(345, "GBM Delay OFF 7", eeprom.data.gbm_delay_off[6], 0)
SV(346, "GBM Delay OFF 8", eeprom.data.gbm_delay_off[7], 0)
SV(347, "GBM Delay OFF 9", eeprom.data.gbm_delay_off[8], 0)
SV(348, "GBM Delay OFF 10", eeprom.data.gbm_delay_off[9], 0)
SV(349, "GBM Delay OFF 11", eeprom.data.gbm_delay_off[10], 0)
SV(350, "GBM Delay OFF 12", eeprom.data.gbm_delay_off[11], 0)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
uint8_t gbm_mode;
uint8_t gbm_threshold_on[12];
uint8_t gbm_threshold_off[12];
uint8_t gbm_delay_on[12];
uint8_t gbm_delay_off[12];
#endif

/*
 *	EEPROM Status Variable Definition
 */
#ifdef EEPROM_STATUS_CFG
//uint8_t test_status;
#endif

/*
 *	EEPROM Configuration Variable Default Configuration
 */
#ifdef EEPROM_DEFAULT
.gbm_mode = GBM_MODE_NORMAL,
.gbm_threshold_on = { 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15 },
.gbm_threshold_off = { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 },
.gbm_delay_on = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5 },
.gbm_delay_off = { 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40 },
#endif

/*
 *	EEPROM Status Variable Default Configuration
 */
#ifdef EEPROM_STATUS_DEFAULT
//.test_status = 1,
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
//SV_CMD_CALLBACK(ln_sv_cmd_callback)
#endif


#else
/************************************************************************/
/* Module Header File                                                   */
/************************************************************************/
#ifndef gbm_H_
#define gbm_H_

PROCESS_NAME(gbm_process);
void gbm_init(void);
void gbm_helper_multi_threshold_on(void);
void gbm_helper_multi_threshold_off(void);
void gbm_helper_multi_delay_on(void);
void gbm_helper_multi_delay_off(void);

extern volatile uint8_t gbm_adc_phase;
extern int8_t gbm_value_act[12];
extern uint8_t gbm_avg[12];
extern uint16_t gbm_avg_int[12];
extern uint16_t gbm_register_filt;
extern uint16_t gbm_register_filt_filt;

extern uint8_t gbm_track_select_L;
extern uint8_t gbm_track_select_H;
extern uint8_t gbm_temp_multi;

#define GBM_TIMECONST 128

/************************************************************************/
/* Config Flags                                                         */
/************************************************************************/
#define GBM_MODE_INIT 0
#define GBM_MODE_NORMAL	1
#define GBM_MODE_SBK 2
#define GBM_MODE_FSZ 3

#endif /* gbm_H_ */
#endif