#include "offline_foc_detect.h"

#include "app.h"
#include "ch.h"
#include "commands.h"
#include "conf_general.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "mempools.h"

#include <math.h>

#ifdef HW_OFFLINE_FOC_DETECT_BOOT

#ifndef HW_OFFLINE_FOC_DETECT_LATCH_DELAY_MS
#define HW_OFFLINE_FOC_DETECT_LATCH_DELAY_MS			50
#endif

#ifndef HW_OFFLINE_FOC_DETECT_LATCH_SAMPLE_DELAY_MS
#define HW_OFFLINE_FOC_DETECT_LATCH_SAMPLE_DELAY_MS		5
#endif

#ifndef HW_OFFLINE_FOC_DETECT_LATCH_SAMPLES
#define HW_OFFLINE_FOC_DETECT_LATCH_SAMPLES				12
#endif

#ifndef HW_OFFLINE_FOC_DETECT_LATCH_MIN_LOW
#define HW_OFFLINE_FOC_DETECT_LATCH_MIN_LOW				10
#endif

#ifndef HW_OFFLINE_FOC_DETECT_START_DELAY_MS
#define HW_OFFLINE_FOC_DETECT_START_DELAY_MS			3000
#endif

#ifndef HW_OFFLINE_FOC_DETECT_POWER_LOSS_W
#define HW_OFFLINE_FOC_DETECT_POWER_LOSS_W				30.0
#endif

#ifndef HW_OFFLINE_FOC_DETECT_L_IN_CURRENT_MIN
#define HW_OFFLINE_FOC_DETECT_L_IN_CURRENT_MIN			0.0
#endif

#ifndef HW_OFFLINE_FOC_DETECT_L_IN_CURRENT_MAX
#define HW_OFFLINE_FOC_DETECT_L_IN_CURRENT_MAX			0.0
#endif

#ifndef HW_OFFLINE_FOC_DETECT_OPENLOOP_RPM
#define HW_OFFLINE_FOC_DETECT_OPENLOOP_RPM				0.0
#endif

#ifndef HW_OFFLINE_FOC_DETECT_SL_ERPM
#define HW_OFFLINE_FOC_DETECT_SL_ERPM					0.0
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_TRIGGER_BOOT
#define HW_OFFLINE_FOC_DETECT_PPM_TRIGGER_BOOT			1
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_LATCH_DELAY_MS
#define HW_OFFLINE_FOC_DETECT_PPM_LATCH_DELAY_MS		250
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_SAMPLE_DELAY_MS
#define HW_OFFLINE_FOC_DETECT_PPM_SAMPLE_DELAY_MS		10
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_SAMPLES
#define HW_OFFLINE_FOC_DETECT_PPM_SAMPLES				20
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_MIN_HIGH
#define HW_OFFLINE_FOC_DETECT_PPM_MIN_HIGH				18
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_HIGH_THRESHOLD
#define HW_OFFLINE_FOC_DETECT_PPM_HIGH_THRESHOLD		0.95f
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_MID_THRESHOLD
#define HW_OFFLINE_FOC_DETECT_PPM_MID_THRESHOLD			(1.0f / 3.0f)
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_HIGH_MID_THRESHOLD
#define HW_OFFLINE_FOC_DETECT_PPM_HIGH_MID_THRESHOLD	(2.0f / 3.0f)
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_POWER_LOSS_LOW_W
#define HW_OFFLINE_FOC_DETECT_PPM_POWER_LOSS_LOW_W		100.0f
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_POWER_LOSS_MID_W
#define HW_OFFLINE_FOC_DETECT_PPM_POWER_LOSS_MID_W		200.0f
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_POWER_LOSS_HIGH_W
#define HW_OFFLINE_FOC_DETECT_PPM_POWER_LOSS_HIGH_W		300.0f
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_CURRENT_LIMIT_LOW_A
#define HW_OFFLINE_FOC_DETECT_PPM_CURRENT_LIMIT_LOW_A	60.0f
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_CURRENT_LIMIT_MID_A
#define HW_OFFLINE_FOC_DETECT_PPM_CURRENT_LIMIT_MID_A	70.0f
#endif

#ifndef HW_OFFLINE_FOC_DETECT_PPM_CURRENT_LIMIT_HIGH_A
#define HW_OFFLINE_FOC_DETECT_PPM_CURRENT_LIMIT_HIGH_A	80.0f
#endif

#ifndef HW_OFFLINE_FOC_DETECT_BEEP_FREQ_HZ
#define HW_OFFLINE_FOC_DETECT_BEEP_FREQ_HZ				659.25f
#endif

#ifndef HW_OFFLINE_FOC_DETECT_BEEP_TIME_S
#define HW_OFFLINE_FOC_DETECT_BEEP_TIME_S				0.20f
#endif

#ifndef HW_OFFLINE_FOC_DETECT_BEEP_GAP_MS
#define HW_OFFLINE_FOC_DETECT_BEEP_GAP_MS				200
#endif

#ifndef HW_OFFLINE_FOC_DETECT_BEEP_VOLTAGE
#define HW_OFFLINE_FOC_DETECT_BEEP_VOLTAGE				6.0f
#endif

static THD_WORKING_AREA(offline_foc_detect_thread_wa, 1024);
static bool offline_foc_detect_requested = false;
static bool offline_foc_detect_started = false;
static float offline_foc_detect_power_loss_w = HW_OFFLINE_FOC_DETECT_POWER_LOSS_W;
static float offline_foc_detect_current_limit_a = 0.0f;

static void offline_foc_detect_prepare_mcconf(mc_configuration *mcconf) {
	if (fabsf((float)HW_OFFLINE_FOC_DETECT_L_IN_CURRENT_MIN) > 0.001f) {
		mcconf->l_in_current_min = HW_OFFLINE_FOC_DETECT_L_IN_CURRENT_MIN;
	}

	if (fabsf((float)HW_OFFLINE_FOC_DETECT_L_IN_CURRENT_MAX) > 0.001f) {
		mcconf->l_in_current_max = HW_OFFLINE_FOC_DETECT_L_IN_CURRENT_MAX;
	}

	if (fabsf((float)HW_OFFLINE_FOC_DETECT_OPENLOOP_RPM) > 0.001f) {
		mcconf->foc_openloop_rpm = HW_OFFLINE_FOC_DETECT_OPENLOOP_RPM;
	}

	if (fabsf((float)HW_OFFLINE_FOC_DETECT_SL_ERPM) > 0.001f) {
		mcconf->foc_sl_erpm = HW_OFFLINE_FOC_DETECT_SL_ERPM;
	}
}

static bool offline_foc_detect_validate_conf(const mc_configuration *mcconf) {
	return isfinite(mcconf->foc_motor_r) &&
			isfinite(mcconf->foc_motor_l) &&
			isfinite(mcconf->foc_motor_flux_linkage) &&
			isfinite(mcconf->foc_current_kp) &&
			isfinite(mcconf->foc_current_ki) &&
			isfinite(mcconf->foc_observer_gain) &&
			mcconf->foc_motor_r > 0.0f &&
			mcconf->foc_motor_l > 0.0f &&
			mcconf->foc_motor_flux_linkage > 0.0f &&
			mcconf->foc_current_kp > 0.0f &&
			mcconf->foc_current_ki > 0.0f &&
			mcconf->foc_observer_gain > 0.0f;
}

static void offline_foc_detect_beep_done(void) {
	for (int i = 0;i < 2;i++) {
		if (!mcpwm_foc_beep(HW_OFFLINE_FOC_DETECT_BEEP_FREQ_HZ,
				HW_OFFLINE_FOC_DETECT_BEEP_TIME_S,
				HW_OFFLINE_FOC_DETECT_BEEP_VOLTAGE)) {
			break;
		}

		if (i < 1) {
			chThdSleepMilliseconds(HW_OFFLINE_FOC_DETECT_BEEP_GAP_MS);
		}
	}

	mcpwm_foc_release_motor();
	chThdSleepMilliseconds(2);
}

static void offline_foc_detect_request(float power_loss_w, float current_limit_a) {
	offline_foc_detect_requested = true;
	offline_foc_detect_power_loss_w = power_loss_w;
	offline_foc_detect_current_limit_a = current_limit_a;
}

static float offline_foc_detect_ppm_power_loss(float *current_limit_a) {
#if HW_OFFLINE_FOC_DETECT_PPM_TRIGGER_BOOT
	const app_configuration *appconf = app_get_configuration();
	if (!appconf) {
		return 0.0f;
	}

	if (appconf->app_to_use != APP_PPM &&
			appconf->app_to_use != APP_PPM_UART) {
		return 0.0f;
	}

	bool app_output_was_disabled = app_is_output_disabled();
	if (!app_output_was_disabled) {
		app_disable_output(-1);
	}

	chThdSleepMilliseconds(HW_OFFLINE_FOC_DETECT_PPM_LATCH_DELAY_MS);

	int low_cnt = 0;
	int mid_cnt = 0;
	int high_cnt = 0;
	for (int i = 0;i < HW_OFFLINE_FOC_DETECT_PPM_SAMPLES;i++) {
		float level = app_ppm_get_decoded_level();

		if (level >= HW_OFFLINE_FOC_DETECT_PPM_MID_THRESHOLD) {
			low_cnt++;
		}

		if (level >= HW_OFFLINE_FOC_DETECT_PPM_HIGH_MID_THRESHOLD) {
			mid_cnt++;
		}

		if (level >= HW_OFFLINE_FOC_DETECT_PPM_HIGH_THRESHOLD) {
			high_cnt++;
		}

		chThdSleepMilliseconds(HW_OFFLINE_FOC_DETECT_PPM_SAMPLE_DELAY_MS);
	}

	float power_loss_w = 0.0f;
	if (high_cnt >= HW_OFFLINE_FOC_DETECT_PPM_MIN_HIGH) {
		power_loss_w = HW_OFFLINE_FOC_DETECT_PPM_POWER_LOSS_HIGH_W;
		*current_limit_a = HW_OFFLINE_FOC_DETECT_PPM_CURRENT_LIMIT_HIGH_A;
	} else if (mid_cnt >= HW_OFFLINE_FOC_DETECT_PPM_MIN_HIGH) {
		power_loss_w = HW_OFFLINE_FOC_DETECT_PPM_POWER_LOSS_MID_W;
		*current_limit_a = HW_OFFLINE_FOC_DETECT_PPM_CURRENT_LIMIT_MID_A;
	} else if (low_cnt >= HW_OFFLINE_FOC_DETECT_PPM_MIN_HIGH) {
		power_loss_w = HW_OFFLINE_FOC_DETECT_PPM_POWER_LOSS_LOW_W;
		*current_limit_a = HW_OFFLINE_FOC_DETECT_PPM_CURRENT_LIMIT_LOW_A;
	}

	if (power_loss_w <= 0.0f && !app_output_was_disabled) {
		app_disable_output(0);
	}

	return power_loss_w;
#else
	(void)current_limit_a;
	return 0.0f;
#endif
}

static THD_FUNCTION(offline_foc_detect_thread, arg) {
	(void)arg;

	chRegSetThreadName("Offline FOC");

	mc_configuration *mcconf_old = mempools_alloc_mcconf();
	mc_configuration *mcconf_work = mempools_alloc_mcconf();

	if (!mcconf_old || !mcconf_work) {
		commands_printf("Offline FOC detect aborted: mcconf allocation failed.");
		goto cleanup;
	}

	app_disable_output(-1);
	LED_RED_ON();

	*mcconf_old = *mc_interface_get_configuration();
	*mcconf_work = *mcconf_old;
	offline_foc_detect_prepare_mcconf(mcconf_work);
	mc_interface_set_configuration(mcconf_work);

	chThdSleepMilliseconds(HW_OFFLINE_FOC_DETECT_START_DELAY_MS);

	int res = conf_general_detect_apply_all_foc(offline_foc_detect_power_loss_w, false, false);

	if (res < 0) {
		commands_printf("Offline FOC detect failed with result %d.", res);
		mc_interface_set_configuration(mcconf_old);
		goto cleanup;
	}

	*mcconf_work = *mc_interface_get_configuration();

	if (!offline_foc_detect_validate_conf(mcconf_work)) {
		commands_printf("Offline FOC detect aborted: invalid detected values.");
		mc_interface_set_configuration(mcconf_old);
		goto cleanup;
	}

	if (offline_foc_detect_current_limit_a > 0.0f) {
		mcconf_work->l_current_max = offline_foc_detect_current_limit_a;
		mcconf_work->l_current_min = -offline_foc_detect_current_limit_a;
		mcconf_work->l_current_max_scale = 1.0f;
		mcconf_work->l_current_min_scale = 1.0f;
	}

	mc_interface_set_configuration(mcconf_work);

	if (!conf_general_store_mc_configuration(mcconf_work,
			mc_interface_get_motor_thread() == 2)) {
		commands_printf("Offline FOC detect aborted: storing mcconf failed.");
		mc_interface_set_configuration(mcconf_old);
		goto cleanup;
	}

	commands_printf("Offline FOC detect completed and stored.");
	offline_foc_detect_beep_done();

cleanup:
	LED_RED_OFF();
	app_disable_output(0);
	offline_foc_detect_requested = false;
	offline_foc_detect_started = false;

	if (mcconf_old) {
		mempools_free_mcconf(mcconf_old);
	}

	if (mcconf_work) {
		mempools_free_mcconf(mcconf_work);
	}
}

void offline_foc_detect_boot_latch(void) {
	palSetPadMode(HW_OFFLINE_FOC_DETECT_GPIO, HW_OFFLINE_FOC_DETECT_PIN, PAL_MODE_INPUT_PULLUP);

	chThdSleepMilliseconds(HW_OFFLINE_FOC_DETECT_LATCH_DELAY_MS);

	int low_cnt = 0;
	for (int i = 0;i < HW_OFFLINE_FOC_DETECT_LATCH_SAMPLES;i++) {
		if (!palReadPad(HW_OFFLINE_FOC_DETECT_GPIO, HW_OFFLINE_FOC_DETECT_PIN)) {
			low_cnt++;
		}
		chThdSleepMilliseconds(HW_OFFLINE_FOC_DETECT_LATCH_SAMPLE_DELAY_MS);
	}

	palSetPadMode(HW_OFFLINE_FOC_DETECT_GPIO, HW_OFFLINE_FOC_DETECT_PIN, PAL_MODE_INPUT);
	if (low_cnt >= HW_OFFLINE_FOC_DETECT_LATCH_MIN_LOW) {
		offline_foc_detect_request(HW_OFFLINE_FOC_DETECT_POWER_LOSS_W, 0.0f);
	}
}

void offline_foc_detect_start_if_requested(void) {
	if (!offline_foc_detect_requested) {
		float ppm_current_limit_a = 0.0f;
		float ppm_power_loss_w = offline_foc_detect_ppm_power_loss(&ppm_current_limit_a);
		if (ppm_power_loss_w > 0.0f) {
			offline_foc_detect_request(ppm_power_loss_w, ppm_current_limit_a);
		}
	}

	if (!offline_foc_detect_requested || offline_foc_detect_started) {
		return;
	}

	offline_foc_detect_started = true;
	chThdCreateStatic(offline_foc_detect_thread_wa, sizeof(offline_foc_detect_thread_wa),
			NORMALPRIO, offline_foc_detect_thread, NULL);
}

void offline_foc_detect_request_start(void) {
	offline_foc_detect_request(HW_OFFLINE_FOC_DETECT_POWER_LOSS_W, 0.0f);
	offline_foc_detect_start_if_requested();
}

#else

void offline_foc_detect_boot_latch(void) {
}

void offline_foc_detect_start_if_requested(void) {
}

void offline_foc_detect_request_start(void) {
}

#endif
