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
&port_process,
#endif

/*
 *	SV Configuration Table
 *
 *	List of all configuration variables defined by this module
 *
 */
#ifdef SV_CFG
SV(6, "Port Configuration Register", eeprom.port_config, port_update_configuration)
SV_LSB(7, "Port Direction Register L", eeprom.port_dir, port_update_configuration)
SV_MSB(8, "Port Direction Register H", eeprom.port_dir, port_update_configuration)
SV_LSB(9, "Port Input Status L", port_di, 0)
SV_MSB(10, "Port Input Status H", port_di, 0)
SV_LSB(11, "Port Output Status L", port_do_mapped, 0)
SV_MSB(12, "Port Output Status H", port_do_mapped, 0)
SV_LSB(13, "Port Map Direction L", eeprom.port_map_dir, 0)
SV_MSB(14, "Port Map Direction H", eeprom.port_map_dir, 0)
SV_LSB(15, "Port User Command L", port_user, 0)
SV_MSB(16, "Port User Command H", port_user, 0)
SV_LSB(17, "Port PWM Enable L", eeprom.port_pwm_enable, 0)
SV_MSB(18, "Port PWM Enable H", eeprom.port_pwm_enable, 0)
SV_LSB(19, "PWM Port Select", pwm_parameter_select, 0)
SV_MSB(20, "PWM Port Select", pwm_parameter_select, 0)
SV(21, "PWM Port Target", pwm_target_temp, pwm_target_parameter_update)
SV(22, "PWM Port Delta", pwm_delta_temp, pwm_delta_parameter_update)
SV_LSB(23, "PWM Update Trigger L", pwm_update_trig, 0)
SV_MSB(24, "PWM Update Trigger H", pwm_update_trig, 0)
SV(25, "PWM Target 1", pwm_target[0], 0)
SV(26, "PWM Target 2", pwm_target[1], 0)
SV(27, "PWM Target 3", pwm_target[2], 0)
SV(28, "PWM Target 4", pwm_target[3], 0)
SV(29, "PWM Target 5", pwm_target[4], 0)
SV(30, "PWM Target 6", pwm_target[5], 0)
SV(31, "PWM Target 7", pwm_target[6], 0)
SV(32, "PWM Target 8", pwm_target[7], 0)
SV(33, "PWM Target 9", pwm_target[8], 0)
SV(34, "PWM Target 10", pwm_target[9], 0)
SV(35, "PWM Target 11", pwm_target[10], 0)
SV(36, "PWM Target 12", pwm_target[11], 0)
SV(37, "PWM Target 13", pwm_target[12], 0)
SV(38, "PWM Target 14", pwm_target[13], 0)
SV(39, "PWM Target 15", pwm_target[14], 0)
SV(40, "PWM Target 16", pwm_target[15], 0)
SV(41, "PWM Delta 1", eeprom.pwm_delta[0], 0)
SV(42, "PWM Delta 2", eeprom.pwm_delta[1], 0)
SV(43, "PWM Delta 3", eeprom.pwm_delta[2], 0)
SV(44, "PWM Delta 4", eeprom.pwm_delta[3], 0)
SV(45, "PWM Delta 5", eeprom.pwm_delta[4], 0)
SV(46, "PWM Delta 6", eeprom.pwm_delta[5], 0)
SV(47, "PWM Delta 7", eeprom.pwm_delta[6], 0)
SV(48, "PWM Delta 8", eeprom.pwm_delta[7], 0)
SV(49, "PWM Delta 9", eeprom.pwm_delta[8], 0)
SV(50, "PWM Delta 10", eeprom.pwm_delta[9], 0)
SV(51, "PWM Delta 11", eeprom.pwm_delta[10], 0)
SV(52, "PWM Delta 12", eeprom.pwm_delta[11], 0)
SV(53, "PWM Delta 13", eeprom.pwm_delta[12], 0)
SV(54, "PWM Delta 14", eeprom.pwm_delta[13], 0)
SV(55, "PWM Delta 15", eeprom.pwm_delta[14], 0)
SV(56, "PWM Delta 16", eeprom.pwm_delta[15], 0)
SV(57, "Port Map Lut 1", eeprom.port_map_lut[0], 0)
SV(58, "Port Map Lut 2", eeprom.port_map_lut[1], 0)
SV(59, "Port Map Lut 3", eeprom.port_map_lut[2], 0)
SV(60, "Port Map Lut 4", eeprom.port_map_lut[3], 0)
SV(61, "Port Map Lut 5", eeprom.port_map_lut[4], 0)
SV(62, "Port Map Lut 6", eeprom.port_map_lut[5], 0)
SV(63, "Port Map Lut 7", eeprom.port_map_lut[6], 0)
SV(64, "Port Map Lut 8", eeprom.port_map_lut[7], 0)
SV(65, "Port Map Lut 9", eeprom.port_map_lut[8], 0)
SV(66, "Port Map Lut 10", eeprom.port_map_lut[9], 0)
SV(67, "Port Map Lut 11", eeprom.port_map_lut[10], 0)
SV(68, "Port Map Lut 12", eeprom.port_map_lut[11], 0)
SV(69, "Port Map Lut 13", eeprom.port_map_lut[12], 0)
SV(70, "Port Map Lut 14", eeprom.port_map_lut[13], 0)
SV(71, "Port Map Lut 15", eeprom.port_map_lut[14], 0)
SV(72, "Port Map Lut 16", eeprom.port_map_lut[15], 0)
SV(73, "Port Map Mux0 1", eeprom.port_map_mux0[0], 0)
SV(74, "Port Map Mux0 2", eeprom.port_map_mux0[1], 0)
SV(75, "Port Map Mux0 3", eeprom.port_map_mux0[2], 0)
SV(76, "Port Map Mux0 4", eeprom.port_map_mux0[3], 0)
SV(77, "Port Map Mux0 5", eeprom.port_map_mux0[4], 0)
SV(78, "Port Map Mux0 6", eeprom.port_map_mux0[5], 0)
SV(79, "Port Map Mux0 7", eeprom.port_map_mux0[6], 0)
SV(80, "Port Map Mux0 8", eeprom.port_map_mux0[7], 0)
SV(81, "Port Map Mux0 9", eeprom.port_map_mux0[8], 0)
SV(82, "Port Map Mux0 10", eeprom.port_map_mux0[9], 0)
SV(83, "Port Map Mux0 11", eeprom.port_map_mux0[10], 0)
SV(84, "Port Map Mux0 12", eeprom.port_map_mux0[11], 0)
SV(85, "Port Map Mux0 13", eeprom.port_map_mux0[12], 0)
SV(86, "Port Map Mux0 14", eeprom.port_map_mux0[13], 0)
SV(87, "Port Map Mux0 15", eeprom.port_map_mux0[14], 0)
SV(88, "Port Map Mux0 16", eeprom.port_map_mux0[15], 0)
SV(89, "Port Map Mux1 1", eeprom.port_map_mux1[0], 0)
SV(90, "Port Map Mux1 2", eeprom.port_map_mux1[1], 0)
SV(91, "Port Map Mux1 3", eeprom.port_map_mux1[2], 0)
SV(92, "Port Map Mux1 4", eeprom.port_map_mux1[3], 0)
SV(93, "Port Map Mux1 5", eeprom.port_map_mux1[4], 0)
SV(94, "Port Map Mux1 6", eeprom.port_map_mux1[5], 0)
SV(95, "Port Map Mux1 7", eeprom.port_map_mux1[6], 0)
SV(96, "Port Map Mux1 8", eeprom.port_map_mux1[7], 0)
SV(97, "Port Map Mux1 9", eeprom.port_map_mux1[8], 0)
SV(98, "Port Map Mux1 10", eeprom.port_map_mux1[9], 0)
SV(99, "Port Map Mux1 11", eeprom.port_map_mux1[10], 0)
SV(100, "Port Map Mux1 12", eeprom.port_map_mux1[11], 0)
SV(101, "Port Map Mux1 13", eeprom.port_map_mux1[12], 0)
SV(102, "Port Map Mux1 14", eeprom.port_map_mux1[13], 0)
SV(103, "Port Map Mux1 15", eeprom.port_map_mux1[14], 0)
SV(104, "Port Map Mux1 16", eeprom.port_map_mux1[15], 0)
SV(105, "Port Map Mux2 1", eeprom.port_map_mux2[0], 0)
SV(106, "Port Map Mux2 2", eeprom.port_map_mux2[1], 0)
SV(107, "Port Map Mux2 3", eeprom.port_map_mux2[2], 0)
SV(108, "Port Map Mux2 4", eeprom.port_map_mux2[3], 0)
SV(109, "Port Map Mux2 5", eeprom.port_map_mux2[4], 0)
SV(110, "Port Map Mux2 6", eeprom.port_map_mux2[5], 0)
SV(111, "Port Map Mux2 7", eeprom.port_map_mux2[6], 0)
SV(112, "Port Map Mux2 8", eeprom.port_map_mux2[7], 0)
SV(113, "Port Map Mux2 9", eeprom.port_map_mux2[8], 0)
SV(114, "Port Map Mux2 10", eeprom.port_map_mux2[9], 0)
SV(115, "Port Map Mux2 11", eeprom.port_map_mux2[10], 0)
SV(116, "Port Map Mux2 12", eeprom.port_map_mux2[11], 0)
SV(117, "Port Map Mux2 13", eeprom.port_map_mux2[12], 0)
SV(118, "Port Map Mux2 14", eeprom.port_map_mux2[13], 0)
SV(119, "Port Map Mux2 15", eeprom.port_map_mux2[14], 0)
SV(120, "Port Map Mux2 16", eeprom.port_map_mux2[15], 0)
SV(121, "Port Brightness On 1", eeprom.port_brightness_on[0], 0)
SV(122, "Port Brightness On 2", eeprom.port_brightness_on[1], 0)
SV(123, "Port Brightness On 3", eeprom.port_brightness_on[2], 0)
SV(124, "Port Brightness On 4", eeprom.port_brightness_on[3], 0)
SV(125, "Port Brightness On 5", eeprom.port_brightness_on[4], 0)
SV(126, "Port Brightness On 6", eeprom.port_brightness_on[5], 0)
SV(127, "Port Brightness On 7", eeprom.port_brightness_on[6], 0)
SV(128, "Port Brightness On 8", eeprom.port_brightness_on[7], 0)
SV(129, "Port Brightness On 9", eeprom.port_brightness_on[8], 0)
SV(130, "Port Brightness On 10", eeprom.port_brightness_on[9], 0)
SV(131, "Port Brightness On 11", eeprom.port_brightness_on[10], 0)
SV(132, "Port Brightness On 12", eeprom.port_brightness_on[11], 0)
SV(133, "Port Brightness On 13", eeprom.port_brightness_on[12], 0)
SV(134, "Port Brightness On 14", eeprom.port_brightness_on[13], 0)
SV(135, "Port Brightness On 15", eeprom.port_brightness_on[14], 0)
SV(136, "Port Brightness On 16", eeprom.port_brightness_on[15], 0)
SV(137, "Port Brightness Off 1", eeprom.port_brightness_off[0], 0)
SV(138, "Port Brightness Off 2", eeprom.port_brightness_off[1], 0)
SV(139, "Port Brightness Off 3", eeprom.port_brightness_off[2], 0)
SV(140, "Port Brightness Off 4", eeprom.port_brightness_off[3], 0)
SV(141, "Port Brightness Off 5", eeprom.port_brightness_off[4], 0)
SV(142, "Port Brightness Off 6", eeprom.port_brightness_off[5], 0)
SV(143, "Port Brightness Off 7", eeprom.port_brightness_off[6], 0)
SV(144, "Port Brightness Off 8", eeprom.port_brightness_off[7], 0)
SV(145, "Port Brightness Off 9", eeprom.port_brightness_off[8], 0)
SV(146, "Port Brightness Off 10", eeprom.port_brightness_off[9], 0)
SV(147, "Port Brightness Off 11", eeprom.port_brightness_off[10], 0)
SV(148, "Port Brightness Off 12", eeprom.port_brightness_off[11], 0)
SV(149, "Port Brightness Off 13", eeprom.port_brightness_off[12], 0)
SV(150, "Port Brightness Off 14", eeprom.port_brightness_off[13], 0)
SV(151, "Port Brightness Off 15", eeprom.port_brightness_off[14], 0)
SV(152, "Port Brightness Off 16", eeprom.port_brightness_off[15], 0)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
uint8_t port_config;
uint16_t port_dir;
uint16_t port_map_dir;
uint8_t port_map_lut[16];
uint8_t port_map_mux0[16];
uint8_t port_map_mux1[16];
uint8_t port_map_mux2[16];
int8_t port_brightness_on[16];
int8_t port_brightness_off[16];
uint8_t pwm_delta[16];
uint16_t port_pwm_enable;
#endif

/*
 *	EEPROM Status Variable Definition
 */
#ifdef EEPROM_STATUS_CFG
//uint8_t relay_request;
#endif

/*
 *	EEPROM Configuration Variable Default Configuration
 */
#ifdef EEPROM_DEFAULT
.port_config = (1<<PORT_MODE_PULLUP_ENABLE),
.port_dir = 0,
.port_map_dir = 0,
.port_map_lut = {240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 204, 204},
.port_map_mux0 = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
.port_map_mux1 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 129},
.port_map_mux2 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
.port_brightness_on = {DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX, DIMM_RANGE_MAX},
.port_brightness_off = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
.pwm_delta = {PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS, PWM_STEPS},
.port_pwm_enable = 0,
#endif

/*
 *	EEPROM Status Variable Default Configuration
 */
#ifdef EEPROM_STATUS_DEFAULT
//.relay_request = 0,
#endif

/*
 * LN Receive Callback Definition
 *
 * Function to be called when a valid packet was received
 */
#ifdef LN_RX_CALLBACK
LN_RX_CALLBACK(port_ln_callback)
#endif

#else
/************************************************************************/
/* Module Header File                                                   */
/************************************************************************/
#ifndef PORT_H_
#define PORT_H_

#define MAP_BITS(SRC_REG, DEST_REG, SRC_BIT, DEST_BIT) if (SRC_REG&(1<<(SRC_BIT))) DEST_REG |= (1<<(DEST_BIT)); else DEST_REG &= ~(1<<(DEST_BIT))

#define PORT_MODE_PULLUP_ENABLE		0
#define PORT_MODE_PWM_UPDATE_CONT	1

#define PWM_PORT_COUNT	16

#define PWM_STEPS           64L
#define DIMM_RANGE_MIN		0
#define DIMM_RANGE_MAX		31

typedef struct t_pwm_port
{
	int8_t dimm_current;
	int8_t dimm_target;
	uint8_t dimm_delta;
	uint8_t pwm_current;
} t_pwm_port;

void port_init(void);
void port_update_configuration(void);
void port_update_mapping(void);
void port_di_init(void);

void port_do_mapping(void);
uint16_t port_pin_status(void);

void pwm_init(void);
void pwm_step(void);
void pwm_update_bitslices(void);

void pwm_target_parameter_update(void);
void pwm_delta_parameter_update(void);

void port_ln_callback(lnMsg *LnPacket);

extern uint8_t port[4][8];

extern uint16_t port_do;
extern uint16_t port_di;
extern uint16_t port_di_mapped;
extern uint16_t port_do_mapped;
extern uint16_t port_user;
extern uint8_t port_blink_phase;

extern uint8_t pwm_gamma_4b6b[16];
extern uint8_t pwm_gamma_5b6b[32];
extern uint8_t pwm_bitslice[6][4];
extern uint8_t pwm_bitslice_shadow[6][4];
extern volatile uint8_t pwm_bitslice_update_needed;

extern t_pwm_port pwm_port[PWM_PORT_COUNT];
extern int8_t pwm_target[PWM_PORT_COUNT];
extern uint16_t pwm_update_trig;
extern uint16_t pwm_at_setpoint;
extern uint8_t pwm_target_temp;
extern uint8_t pwm_delta_temp;
extern uint16_t pwm_parameter_select;

PROCESS_NAME(port_process);

#endif /* PORT_H_ */
#endif