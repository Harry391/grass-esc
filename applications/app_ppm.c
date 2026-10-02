/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#pragma GCC optimize ("Os")

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "servo_dec.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "comm_can.h"
#include <math.h>

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_PULSES_WITHOUT_POWER		50

#ifdef HW_PPM_PID_DUTY_CURRENT
/* LS-style replacement for the PPM speed-PID modes. */
#define DUTY_CURRENT_DEADBAND			0.01f
#define DUTY_CURRENT_TAPER_WINDOW		0.10f
#define DUTY_CURRENT_BRAKE_CURRENT		2.5f
#define DUTY_CURRENT_BRAKE_MIN_ERPM		800.0f
#endif

// Threads
static THD_FUNCTION(ppm_thread, arg);
__attribute__((section(".ram4"))) static THD_WORKING_AREA(ppm_thread_wa, 512);
static thread_t *ppm_tp;
static volatile bool ppm_rx = false;

// Private functions
static void servodec_func(void);
#ifdef HW_PPM_PID_DUTY_CURRENT
static float duty_current_command(float throttle, float duty_now, float rpm_now,
		const volatile mc_configuration *mcconf);
#endif

// Private variables
static volatile bool is_running = false;
static volatile bool stop_now = true;
static volatile ppm_config config;
static volatile int pulses_without_power = 0;
static float input_val = 0.0;
static volatile float direction_hyst = 0;
static volatile bool ppm_detached = false;
static volatile float ppm_override = 0.0;
static volatile bool direction_switch_allowed = false;
static volatile int direction_hyst_active_dir = 0;
static volatile int direction_hyst_pending_dir = 0;
static volatile bool direction_hyst_idle_done = false;

// Private functions

void app_ppm_configure(ppm_config *conf) {
	config = *conf;
	pulses_without_power = 0;
	direction_switch_allowed = false;
	direction_hyst_active_dir = 0;
	direction_hyst_pending_dir = 0;
	direction_hyst_idle_done = false;

	if (is_running) {
		servodec_set_pulse_options(config.pulse_start, config.pulse_end, config.median_filter);
	}

	direction_hyst = config.max_erpm_for_dir * 0.20;
}

void app_ppm_start(void) {
	stop_now = false;
	chThdCreateStatic(ppm_thread_wa, sizeof(ppm_thread_wa), NORMALPRIO, ppm_thread, NULL);
}

void app_ppm_stop(void) {
	stop_now = true;

	if (is_running) {
		chEvtSignalI(ppm_tp, (eventmask_t) 1);
		servodec_stop();
	}

	while(is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_ppm_get_decoded_level(void) {
	return input_val;
}

void app_ppm_detach(bool detach) {
	ppm_detached = detach;
}

void app_ppm_override(float val) {
	ppm_override = val;
}

static void servodec_func(void) {
	ppm_rx = true;
	chSysLockFromISR();
	chEvtSignalI(ppm_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

#ifdef HW_PPM_PID_DUTY_CURRENT
static float duty_current_command(float throttle, float duty_now, float rpm_now,
		const volatile mc_configuration *mcconf) {
	const float throttle_abs = fabsf(throttle);
	float current_cmd = 0.0f;

	if (throttle_abs < 0.001f) {
		if (fabsf(rpm_now) > DUTY_CURRENT_BRAKE_MIN_ERPM) {
			current_cmd = rpm_now > 0.0f ?
					-DUTY_CURRENT_BRAKE_CURRENT : DUTY_CURRENT_BRAKE_CURRENT;
		}
	} else {
		const float direction = throttle > 0.0f ? 1.0f : -1.0f;

		if ((rpm_now * direction) < -DUTY_CURRENT_BRAKE_MIN_ERPM) {
			current_cmd = rpm_now > 0.0f ?
					-DUTY_CURRENT_BRAKE_CURRENT : DUTY_CURRENT_BRAKE_CURRENT;
		} else {
			const float duty_target = throttle_abs * mcconf->l_max_duty;
			const float duty_along_target = duty_now * direction;
			const float duty_error = duty_target - duty_along_target;

			if (duty_error < -DUTY_CURRENT_DEADBAND) {
				current_cmd = -direction * DUTY_CURRENT_BRAKE_CURRENT;
			} else if (duty_error > DUTY_CURRENT_DEADBAND) {
				float taper_window = duty_target * 0.5f;
				utils_truncate_number(&taper_window, 0.02f, DUTY_CURRENT_TAPER_WINDOW);

				float taper = (duty_error - DUTY_CURRENT_DEADBAND) / taper_window;
				utils_truncate_number(&taper, 0.0f, 1.0f);

				const float current_limit = direction > 0.0f ?
						mcconf->lo_current_max : fabsf(mcconf->lo_current_min);
				current_cmd = direction * throttle_abs * current_limit * taper;
			}
		}
	}

	utils_truncate_number(&current_cmd, mcconf->lo_current_min, mcconf->lo_current_max);
	return current_cmd;
}
#endif

static THD_FUNCTION(ppm_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_PPM");
	ppm_tp = chThdGetSelfX();

	servodec_set_pulse_options(config.pulse_start, config.pulse_end, config.median_filter);
	servodec_init(servodec_func);
	is_running = true;

	for(;;) {
		chEvtWaitAnyTimeout((eventmask_t)1, MS2ST(2));

		if (stop_now) {
			is_running = false;
			return;
		}

		if (ppm_rx) {
			ppm_rx = false;
			timeout_reset();
		}

		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		const float rpm_now = mc_interface_get_rpm();
		float servo_val = servodec_get_servo(0);

		if (ppm_detached) {
			servo_val = ppm_override;
		}

		float servo_ms = utils_map(servo_val, -1.0, 1.0, config.pulse_start, config.pulse_end);

		static bool servoError = false;

		switch (config.ctrl_type) {
		case PPM_CTRL_TYPE_CURRENT_NOREV:
		case PPM_CTRL_TYPE_DUTY_NOREV:
		case PPM_CTRL_TYPE_PID_NOREV:
		case PPM_CTRL_TYPE_PID_POSITION_360:
			input_val = servo_val;
			servo_val += 1.0;
			servo_val /= 2.0;
			break;

		default:
			// Mapping with respect to center pulsewidth
			if (servo_ms < config.pulse_center) {
				servo_val = utils_map(servo_ms, config.pulse_start,
						config.pulse_center, -1.0, 0.0);
			} else {
				servo_val = utils_map(servo_ms, config.pulse_center,
						config.pulse_end, 0.0, 1.0);
			}
			input_val = servo_val;
			break;
		}
		// All pins and buttons are still decoded for debugging, even
		// when output is disabled.
		if (app_is_output_disabled()) {
			continue;
		}

		if (timeout_has_timeout() || servodec_get_time_since_update() > timeout_get_timeout_msec()) {
			pulses_without_power = 0;
			servoError = true;
			float timeoutCurrent = timeout_get_brake_current();
			mc_interface_set_brake_current(timeoutCurrent);
			if(config.multi_esc){
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current_brake(msg->id, timeoutCurrent);
					}
				}
			}
			continue;
		} else if (mc_interface_get_fault() != FAULT_CODE_NONE && config.safe_start != SAFE_START_NO_FAULT){
			pulses_without_power = 0;
		}

		// Apply deadband
		utils_deadband(&servo_val, config.hyst, 1.0);

		// Apply throttle curve
		servo_val = utils_throttle_curve(servo_val, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		// Apply ramping
		static systime_t last_time = 0;
		static float servo_val_ramp = 0.0;
		float ramp_time = fabsf(servo_val) > fabsf(servo_val_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		// TODO: Remember what this was about?
//		if (fabsf(servo_val) > 0.001) {
//			ramp_time = fminf(config.ramp_time_pos, config.ramp_time_neg);
//		}

		const float dt = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / 1000.0;
		last_time = chVTGetSystemTimeX();

		if (ramp_time > 0.01) {
			const float ramp_step = dt / ramp_time;
			utils_step_towards(&servo_val_ramp, servo_val, ramp_step);
			servo_val = servo_val_ramp;
		}

		float current = 0;
		bool current_mode = false;
		bool current_mode_brake = false;
		bool send_current = false;
		bool send_duty = false;
#ifdef HW_PPM_PID_DUTY_CURRENT
		bool duty_current_mode = false;
		bool duty_current_keep_alive = false;
#endif
		float rpm_local = mc_interface_get_rpm();
		float rpm_lowest = rpm_local;
		float rpm_highest = rpm_local;

		switch (config.ctrl_type) {
		case PPM_CTRL_TYPE_CURRENT_BRAKE_REV_HYST: {
			current_mode = true;

			const float rpm_abs = fabsf(rpm_local);
			const float switch_rpm_low = fmaxf(config.max_erpm_for_dir - direction_hyst, 0.0);
			const float switch_rpm_high = config.max_erpm_for_dir + direction_hyst;
			int requested_dir = 0;

			if (servo_val > 0.0) {
				requested_dir = 1;
			} else if (servo_val < 0.0) {
				requested_dir = -1;
			}

			if (config.max_erpm_for_dir <= 0.0) {
				direction_switch_allowed = rpm_abs < 50.0;
			} else if (direction_switch_allowed) {
				if (rpm_abs > switch_rpm_high) {
					direction_switch_allowed = false;
				}
			} else if (rpm_abs < switch_rpm_low) {
				direction_switch_allowed = true;
			}

			if (direction_hyst_active_dir == 0 && direction_hyst_pending_dir == 0 &&
					requested_dir != 0) {
				direction_hyst_active_dir = requested_dir;
			}

			if (requested_dir == 0) {
				if (direction_hyst_pending_dir != 0 && direction_switch_allowed) {
					direction_hyst_idle_done = true;
				}
			} else if (direction_hyst_pending_dir != 0 &&
					requested_dir == direction_hyst_active_dir) {
				direction_hyst_pending_dir = 0;
				direction_hyst_idle_done = false;
			}

			if (requested_dir != 0 && direction_hyst_active_dir == 0) {
				direction_hyst_active_dir = requested_dir;
			}

			if (requested_dir != 0 &&
					requested_dir == direction_hyst_active_dir &&
					direction_hyst_pending_dir == 0) {
				if (requested_dir > 0) {
					current = servo_val * mcconf->lo_current_max;
				} else {
					current = servo_val * fabsf(mcconf->lo_current_min);
				}
			} else if (requested_dir != 0) {
				if (direction_hyst_pending_dir == 0 ||
						direction_hyst_pending_dir != requested_dir) {
					direction_hyst_pending_dir = requested_dir;
					direction_hyst_idle_done = false;
				}

				if (direction_switch_allowed && direction_hyst_idle_done) {
					direction_hyst_active_dir = requested_dir;
					direction_hyst_pending_dir = 0;
					direction_hyst_idle_done = false;

					if (requested_dir > 0) {
						current = servo_val * mcconf->lo_current_max;
					} else {
						current = servo_val * fabsf(mcconf->lo_current_min);
					}
				} else {
					current_mode_brake = true;
					current = fabsf(servo_val * mcconf->lo_current_min);
				}
			}

			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}

			break;
		}
		case PPM_CTRL_TYPE_CURRENT:
		case PPM_CTRL_TYPE_CURRENT_NOREV:
			current_mode = true;
			if ((servo_val >= 0.0 && rpm_now >= 0.0) || (servo_val < 0.0 && rpm_now <= 0.0)) { //Accelerate
				current = servo_val * mcconf->lo_current_max;
			} else { //Brake
				current = servo_val * fabsf(mcconf->lo_current_min);
			}

			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}
			break;

		case PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE:
		case PPM_CTRL_TYPE_CURRENT_SMART_REV:
			current_mode = true;
			current_mode_brake = servo_val < 0.0;

			if (servo_val >= 0.0 && rpm_now > 0.0) { //Positive input AND going forward = accelerating
				current = servo_val * mcconf->lo_current_max;
			} else { //Negative input OR going backwards = brake (no reverse allowed in those control types)
				current = fabsf(servo_val * mcconf->lo_current_min);
			}

			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}
			break;

		case PPM_CTRL_TYPE_DUTY:
		case PPM_CTRL_TYPE_DUTY_NOREV:
			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}

			if (!(pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start)) {
				mc_interface_set_duty(utils_map(servo_val, -1.0, 1.0, -mcconf->l_max_duty, mcconf->l_max_duty));
				send_duty = true;
			}
			break;
		case PPM_CTRL_TYPE_PID:
		case PPM_CTRL_TYPE_PID_NOREV:
		#ifdef HW_PPM_PID_DUTY_CURRENT
			current_mode = true;
			duty_current_mode = true;
			duty_current_keep_alive = fabsf(servo_val) >= 0.001f;
			current = duty_current_command(servo_val,
					mc_interface_get_duty_cycle_now(), rpm_now, mcconf);

			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}
		#else
			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}

			if (!(pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start)) {
				mc_interface_set_pid_speed(servo_val * config.pid_max_erpm);
				send_current = true;
			}
		#endif
			break;

		case PPM_CTRL_TYPE_PID_POSITION_180: // -180 to 180. center ppm safestart
		case PPM_CTRL_TYPE_PID_POSITION_360: // 0 to +360. minimum ppm safestart
			if (fabsf(servo_val) < 0.02) {
				pulses_without_power++;
			}

			float angle;
			if (config.ctrl_type == PPM_CTRL_TYPE_PID_POSITION_180) {
				angle = (servo_val * 180); // -1 <> +1
			} else {
				angle = (servo_val * 360); // 0 <> +1
			}
			utils_norm_angle(&angle);
			if (!(pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start)) {
				// try to more intelligently safe start by waiting until 
				// ppm "angle" is close to motor angle to go into position mode.
				if (mc_interface_get_control_mode() != CONTROL_MODE_POS){ 	
					if (fabsf(angle - mc_interface_get_pid_pos_now()) < 10) {
						// enable position control.
						mc_interface_set_pid_pos(angle);
					}
					break;
				} else {
					mc_interface_set_pid_pos(angle);
				}
			}
			break;

		default:
			continue;
		}
		//Safe start : If startup, servo timeout or fault, check if idle has been verified for some pulses before driving the motor
		if (pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start) {
			static int pulses_without_power_before = 0;
			if (pulses_without_power == pulses_without_power_before) {
				pulses_without_power = 0;
			}
			pulses_without_power_before = pulses_without_power;

			if (servoError){
				continue;
			}
			if (current_mode) {
				current = 0.0;
			}
#ifdef HW_PPM_PID_DUTY_CURRENT
			duty_current_keep_alive = false;
#endif
		} else {
			servoError = false;
		}

		const float duty_now = mc_interface_get_duty_cycle_now();
		float current_highest_abs = fabsf(mc_interface_get_tot_current_directional_filtered());
		float duty_highest_abs = fabsf(duty_now);

		//If multiple VESCs over CAN, store highest/lowest running values of the whole setup
		if (config.multi_esc) {
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					if (fabsf(msg->rpm) < fabsf(rpm_lowest)) {
						rpm_lowest = msg->rpm;
					}

					if (fabsf(msg->rpm) > fabsf(rpm_highest)) {
						rpm_highest = msg->rpm;
					}

					if (fabsf(msg->current) > current_highest_abs) {
						current_highest_abs = fabsf(msg->current);
					}

					if (fabsf(msg->duty) > duty_highest_abs) {
						duty_highest_abs = fabsf(msg->duty);
					}
				}
			}
		}

		if (config.ctrl_type == PPM_CTRL_TYPE_CURRENT_SMART_REV) {
			bool duty_control = false;
			static bool was_duty_control = false;
			static float duty_rev = 0.0;

			if (servo_val < -0.92 && duty_highest_abs < (mcconf->l_min_duty * 1.5) &&
					current_highest_abs < (mcconf->l_current_max * mcconf->l_current_max_scale * 0.7)) {
				duty_control = true;
			}

			if (duty_control || (was_duty_control && servo_val < -0.1)) {
				was_duty_control = true;

				float goal = config.smart_rev_max_duty * -servo_val;
				utils_step_towards(&duty_rev, -goal,
						config.smart_rev_max_duty * dt / config.smart_rev_ramp_time);

				mc_interface_set_duty(duty_rev);

				// Send the same duty cycle to the other controllers
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							comm_can_set_duty(msg->id, duty_rev);
						}
					}
				}

				current_mode = false;
			} else {
				duty_rev = duty_now;
				was_duty_control = false;
			}
		}
		//CTRL TYPE PID_NOREV & DUTY_NOREV : Acting as master, send motor control command to all slave VESCs detected on the CANbus
		if ((send_current || send_duty) && config.multi_esc) {
			float current_filtered = mc_interface_get_tot_current_directional_filtered();
			float duty = mc_interface_get_duty_cycle_now();

			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					if (send_current) {
						comm_can_set_current(msg->id, current_filtered);
					} else if (send_duty) {
						comm_can_set_duty(msg->id, duty);
					}
				}
			}
		}
//CTRL TYPE CURRENT
#ifdef HW_PPM_PID_DUTY_CURRENT
		if (current_mode && duty_current_mode) {
			/* Keep the zero-torque current loop alive while a non-zero duty
			 * target is held, otherwise PWM can stop and repeatedly restart. */
			if (duty_current_keep_alive && mc_interface_get_state() == MC_STATE_RUNNING) {
				mc_interface_set_current_off_delay(0.05f);
			}

			mc_interface_set_current(current);

			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current(msg->id, current);
					}
				}
			}

			current_mode = false;
		}
#endif
		if (current_mode) {
			if (current_mode_brake) { //If braking applied
				mc_interface_set_brake_current(fabsf(current));

				// Send brake command to all ESCs seen recently on the CAN bus
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							comm_can_set_current_brake_rel(msg->id, fabsf(servo_val));
						}
					}
				}
			} else {
				float current_out = current;
				bool is_reverse = false;
				static bool autoTCdisengaged = false;
				if (current_out < 0.0) { // Not braking AND negative current = reverse engaged
					is_reverse = true;
					current_out = -current_out;
					current = -current;
					rpm_local = -rpm_local;
					rpm_lowest = -rpm_lowest;
					rpm_highest = -rpm_highest;
					servo_val = -servo_val;
				}

				// Send acceleration command to all ESCs seen recently on the CAN bus
				if (config.multi_esc) {
					if (config.tc) {
						if(mc_interface_get_fault() != FAULT_CODE_NONE) {
							autoTCdisengaged = true;
						} else if (autoTCdisengaged && rpm_highest < rpm_local + config.tc_max_diff && rpm_lowest > rpm_local - config.tc_max_diff) { //No Fault anymore and no traction control action needed = re-enable traction control
							autoTCdisengaged = false;
						}
					}
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							//Traction Control - Applied to slaves except if a fault has occured on the local VESC (undriven wheel may generate fake RPM)
							if (config.tc && config.tc_max_diff > 1.0 && !autoTCdisengaged) {
								float rpm_tmp = msg->rpm;
								if (is_reverse) {
									rpm_tmp = -rpm_tmp;
								}

								float diff = rpm_tmp - rpm_lowest;
								servo_val = utils_map(diff, 0.0, config.tc_max_diff, servo_val, 0.0);
							}
							//Send motor drive command to slaves
							if (is_reverse) {
								comm_can_set_current_rel(msg->id, -servo_val);
							} else {
								comm_can_set_current_rel(msg->id, servo_val);
							}
						}
					}
					//Traction Control - Applying locally
					if (config.tc && config.tc_max_diff > 1.0) {
						float diff = rpm_local - rpm_lowest;
						current_out = utils_map(diff, 0.0, config.tc_max_diff, current, 0.0);
						if (current_out < mcconf->cc_min_current) {
							current_out = 0.0;
						}
					}
				}
				//Drive local motor
				if (is_reverse) {
					mc_interface_set_current(-current_out);
				} else {
					mc_interface_set_current(current_out);
				}
			}
		}

	}
}

