#include "offline_foc_detect.h"

#include "app.h"
#include "ch.h"
#include "commands.h"
#include "conf_general.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
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

static THD_WORKING_AREA(offline_foc_detect_thread_wa, 1024);
static bool offline_foc_detect_requested = false;
static bool offline_foc_detect_started = false;

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

	int res = conf_general_detect_apply_all_foc(HW_OFFLINE_FOC_DETECT_POWER_LOSS_W, false, false);

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

	mc_interface_set_configuration(mcconf_work);

	if (!conf_general_store_mc_configuration(mcconf_work,
			mc_interface_get_motor_thread() == 2)) {
		commands_printf("Offline FOC detect aborted: storing mcconf failed.");
		mc_interface_set_configuration(mcconf_old);
		goto cleanup;
	}

	commands_printf("Offline FOC detect completed and stored.");

cleanup:
	LED_RED_OFF();
	app_disable_output(0);

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
	offline_foc_detect_requested = low_cnt >= HW_OFFLINE_FOC_DETECT_LATCH_MIN_LOW;
}

void offline_foc_detect_start_if_requested(void) {
	if (!offline_foc_detect_requested || offline_foc_detect_started) {
		return;
	}

	offline_foc_detect_started = true;
	chThdCreateStatic(offline_foc_detect_thread_wa, sizeof(offline_foc_detect_thread_wa),
			NORMALPRIO, offline_foc_detect_thread, NULL);
}

#else

void offline_foc_detect_boot_latch(void) {
}

void offline_foc_detect_start_if_requested(void) {
}

#endif
