//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "diag/Trace.h"

#include "bldc-include.h"
#include "bldc-config.h"
#include "bldc-interrupts.h"
#include "bldc-gpio.h"
#include "bldc-timer.h"
#include "bldc-adc.h"
#include "bldc-adctrigger.h"
#include "bldc-6step.h"
#include "bldc-usart.h"
#include "bldc-dma.h"
#include "bldc-filter.h"
#include "main.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


#define SINE_SAMPLES (144)
static int32_t g_sine_buffer120[SINE_SAMPLES * 2];
static int32_t g_sine_buffer360[SINE_SAMPLES * 2];

BldcFilter g_current_filter;
BldcFilter g_resmsr_filter_0;
BldcFilter g_resmsr_filter_1;
BldcFilter g_resmsr_filter_2;

#define SAMPLE_BUFFER_BITS 7
#define SAMPLE_BUFFER_SIZE (0x1 << SAMPLE_BUFFER_BITS)
#define SAMPLE_BUFFER_MASK (SAMPLE_BUFFER_SIZE - 1)

typedef struct __BLDC_Debug {
	int32_t bemf;
	uint32_t state;
	uint32_t counter;
} BLDC_Debug;

#define DEBUG_PRINT
#undef DEBUG_SAMPLE_BUFFER

#ifdef DEBUG_SAMPLE_BUFFER
static BLDC_Debug g_sample_buffer[SAMPLE_BUFFER_SIZE];
static uint32_t g_debug_counter = 0;
#endif

BLDC_TypeDef g_bldc;
BLDC_TypeDef g_bldc_saved;
static uint32_t pwm_period;
static uint32_t pwm_pulse;
static uint32_t pwm_pulse_us;
static uint32_t amc_switching_hz;
static uint32_t amc_autoreload;
static uint32_t pwm_clock_hz;
static uint32_t com_cycles_max;
static uint32_t resmsr_counter;
static int32_t Vmsr0;
static int32_t Vmsr1;
static int32_t Vmsr2;
static uint64_t jiffies;
static uint32_t injdata[4];


void _fini()
{
}

void bldc_init_sine()
{
	unsigned int i;

	for (i = 0; i < sizeof(g_sine_buffer120)/sizeof(g_sine_buffer120[0]); i++) {
		g_sine_buffer120[i] = 0;
	}
	for (i = 0; i < SINE_SAMPLES; i++) {
		float sine = sin(1.0 * M_PI / 3.0 + i * 1.0 * M_PI / 3.0 / SINE_SAMPLES) * 100.0;
		g_sine_buffer120[i] = sine;
	}

	for (i = 0; i < sizeof(g_sine_buffer360)/sizeof(g_sine_buffer360[0]); i++) {
		g_sine_buffer360[i] = 0;
	}
	for (i = 0; i < SINE_SAMPLES; i++) {
		float sine = (1.0 + sin(1.0 * M_PI / 2.0 + i * 2.0 * M_PI / SINE_SAMPLES)) / 2.0 * 100.0;
		g_sine_buffer360[i] = sine;
	}

}


void bldc_set_substate(BLDC_TypeDef *bldc, uint32_t substate)
{
	__disable_irq();
	bldc->substate = substate;
	__enable_irq();
}

uint32_t bldc_get_substate(BLDC_TypeDef *bldc)
{
	return bldc->substate;
}

void bldc_set_error_substate(BLDC_TypeDef *bldc, uint32_t error)
{
	bldc->error = error;
	if (error)
		bldc_set_substate(bldc, SUBSTATE_ERROR);
}

void bldc_pwm_cc_callback()
{
	BLDC_TypeDef *bldc = &g_bldc;
	int32_t pulse_us = 0;
	int32_t pwm_throttle = 0;
	__disable_irq();
	pwm_period = LL_TIM_OC_GetCompareCH1(TIM3);
	pwm_pulse = LL_TIM_IC_GetCaptureCH2(TIM3);
	__enable_irq();
	pwm_pulse_us = 1000000000 / pwm_clock_hz * pwm_pulse / 1000;

	pulse_us = pwm_pulse_us - THROTTLE_MIN_PWM_DUTY;
	if (pulse_us < 0)
		pulse_us = 0;
	else if (pulse_us > 800)
		pulse_us = 800;

	pwm_throttle = 100 * pulse_us / 800;

	if (pwm_throttle > 0 && jiffies - bldc->bootstrap_time > 25000) {
		bldc->throttle = (MIN_THROTTLE_PERCENT + pwm_throttle * (100 - MIN_THROTTLE_PERCENT) / 100);
	}

#ifdef START_WITH_THROTTLE
	if (bldc_get_substate(bldc) == SUBSTATE_DISABLED && pwm_throttle > 3 && !LL_TIM_IsEnabledCounter(TIM_AMC)) {
		if (jiffies - bldc->last_zerodetected_time < 1000000) {
			bldc_set_substate(bldc, SUBSTATE_MEASUREMENT1);
			bldc->bootstrap_counter = BOOTSTRAP_COUNTER_INIT;
		} else {
			bldc_set_substate(bldc, SUBSTATE_BOOTSTRAP_INIT);
		}
		bldc_6step_start(TIM_AMC);
	} else if (pwm_throttle < 2 && LL_TIM_IsEnabledCounter(TIM_AMC)) {
		bldc_6step_stop(TIM_AMC);
		bldc_set_substate(bldc, SUBSTATE_DISABLED);
	} else if (bldc_get_substate(bldc) == SUBSTATE_ERROR && pwm_throttle < 2) {
		bldc_6step_stop(TIM_AMC);
		bldc_set_substate(bldc, SUBSTATE_DISABLED);
	}
#endif
}


void bldc_emergency_stop()
{
	LL_TIM_DisableAllOutputs(TIM_AMC);
	bldc_6step_stop(TIM_AMC);
}

void bldc_timer_blink_callback()
{
	bldc_gpio_toggle(LL_PORT(LED_WARNING_PIN), LL_PINMASK(LED_WARNING_PIN));
}

void bldc_timer_jiffies_callback()
{
	jiffies++;
}

void bldc_user_button_callback()
{
	bldc_set_substate(&g_bldc, SUBSTATE_BOOTSTRAP_INIT);
	bldc_6step_toggle(TIM_AMC);
	bldc_gpio_activelow_write(LL_PORT(LED_STATUS_PIN), LL_PINMASK(LED_STATUS_PIN), LL_TIM_IsEnabledCounter(TIM_AMC), LED_STATUS_ACTIVE_LOW);
}

void bldc_current_fault_callback()
{
	bldc_set_error_substate(&g_bldc, ERROR_CURRENT_FAULT);
	bldc_emergency_stop();
	printf("Current Fault\n");
}


void bldc_adc_eocs_callback()
{

}


/*
 * return 1 to generate COM event
 * 0 otherwise
 */
int bldc_detect_bemf(BLDC_TypeDef *bldc, int32_t Vh, int32_t Vb, int32_t Vl)
{
	if (bldc_get_substate(bldc) == SUBSTATE_ZERODETECTED) {
		bldc->integral_bemf += bldc->bemf;
		if ((abs(bldc->integral_bemf) > (long)BEMF_INTEGRAL_THRESHOLD)) {
			return 1;
		}
	}

	return 0;
}


void bldc_set_throttle(BLDC_TypeDef *bldc, uint32_t throttle)
{
	uint64_t delta_t = jiffies - bldc->megathrottle_time;
//	float throttle_speed_slope = (THROTTLE_SPEED_HZ_250 - THROTTLE_SPEED_HZ_0) / (250.0);
//	bldc->throttle_speed = THROTTLE_SPEED_HZ_0 + throttle_speed_slope * bldc->rotation_hz;
	float throttle_speed_func = THROTTLE_SPEED_HZ_0 + ((THROTTLE_SPEED_HZ_250 - THROTTLE_SPEED_HZ_0)/ (250.0 * 250.0)) * (bldc->rotation_hz * bldc->rotation_hz);

	bldc->throttle_speed = (throttle_speed_func < THROTTLE_SPEED_HZ_250) ? throttle_speed_func : THROTTLE_SPEED_HZ_250;

	if (delta_t > 100000) {
		/*
		 * This can't be trusted as valid delta_t
		 * Don't adjust the throttle now;
		 */
		goto end;
	}

	if (bldc->megathrottle_current < (int32_t)throttle * 1000000) {
		bldc->megathrottle_current += bldc->throttle_speed * delta_t;
	} else {
		bldc->megathrottle_current -= bldc->throttle_speed * delta_t;
	}
	bldc_6step_set_throttle(TIM_AMC, bldc->megathrottle_current/1000000);
end:
	bldc->megathrottle_time = jiffies;
	return;
}


void bldc_generate_com_event(BLDC_TypeDef *bldc)
{
	LL_TIM_GenerateEvent_COM(TIM_AMC);

	__disable_irq();

	bldc->rotation_hz = amc_switching_hz / (SINE_STATES * MECHANICAL_DEGREES_RATIO * REPETIONS_TO_UPDATE * bldc->counter);
	bldc_set_throttle(bldc, bldc->throttle);
	bldc->output_counter++;

	if ((bldc->output_counter % 289) == 0)
		memcpy(&g_bldc_saved, bldc, sizeof(BLDC_TypeDef));

	bldc->last_counter = bldc->counter;
	bldc->counter = 0UL;
	bldc->zero_detected = 0UL;
	bldc->integral_bemf = 0L;

	bldc->state = (bldc->state + 1) % 6;

	/*
	 * Prepare the preload bits for the next COM event
	 */
	bldc_6step_next(TIM_AMC, (bldc->state + 1) % 6);
	__enable_irq();
}

void bldc_adc_data_acquisition(BLDC_TypeDef *bldc)
{
	__disable_irq();
	injdata[0] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1), LL_ADC_RESOLUTION_12B);
	injdata[1] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2), LL_ADC_RESOLUTION_12B);
	injdata[2] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3), LL_ADC_RESOLUTION_12B);
	injdata[3] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_4), LL_ADC_RESOLUTION_12B);
	__enable_irq();

#ifdef COMPENSATE_DATA_BIAS
	bldc->compdata[0] = Vmsr0 ? injdata[0] * Vmsr0 / Vmsr0 : injdata[0];
	bldc->compdata[1] = Vmsr1 ? injdata[1] * Vmsr0 / Vmsr1 : injdata[1];
	bldc->compdata[2] = Vmsr2 ? injdata[2] * Vmsr0 / Vmsr2 : injdata[2];
	bldc->compdata[3] = injdata[3];
#else
	bldc->compdata[0] = injdata[0];
	bldc->compdata[1] = injdata[1];
	bldc->compdata[2] = injdata[2];
	bldc->compdata[3] = injdata[3];
#endif

}

void bldc_voltage_measurement(BLDC_TypeDef *bldc, BLDC_Measurement *msr)
{
	msr->value = bldc->bemf;
	msr->counter = bldc->counter;
	msr->Vh = bldc->Vh;
	msr->Vb = bldc->Vb;
	msr->Vl = bldc->Vl;
}

void bldc_measure_bemf(BLDC_TypeDef *bldc)
{

	if (bldc->state == 0) {
		/* BEMF CH2 */
		bldc->Vh = bldc->compdata[ADC_A];
		bldc->Vb = bldc->compdata[ADC_B];
		bldc->Vl = bldc->compdata[ADC_C];
	} else if (bldc->state == 1) {
		/* BEMF CH1 */
		bldc->Vh = bldc->compdata[ADC_B];
		bldc->Vb = bldc->compdata[ADC_A];
		bldc->Vl = bldc->compdata[ADC_C];
	} else if (bldc->state == 2) {
		/* BEMF CH3 */
		bldc->Vh = bldc->compdata[ADC_B];
		bldc->Vb = bldc->compdata[ADC_C];
		bldc->Vl = bldc->compdata[ADC_A];
	} else if (bldc->state == 3) {
		/* BEMF CH2 */
		bldc->Vh = bldc->compdata[ADC_C];
		bldc->Vb = bldc->compdata[ADC_B];
		bldc->Vl = bldc->compdata[ADC_A];
	} else if (bldc->state == 4) {
		/* BEMF CH1 */
		bldc->Vh = bldc->compdata[ADC_C];
		bldc->Vb = bldc->compdata[ADC_A];
		bldc->Vl = bldc->compdata[ADC_B];
	} else if (bldc->state == 5) {
		/* BEMF CH3 */
		bldc->Vh = bldc->compdata[ADC_A];
		bldc->Vb = bldc->compdata[ADC_C];
		bldc->Vl = bldc->compdata[ADC_B];
	}
	bldc->bemf = (2 * bldc->Vb  - (bldc->Vh + bldc->Vl)) / 3;
//	bldc->bemf = (2 * (bldc->Vb - 25)  - bldc->Vh) / 3;

	bldc->Vc = bldc->compdata[3];

	/*
	 * Vout = Vcc / 2 + i * (Vcc / 36.7A)
	 *
	 * i = 36.7A * (Vout / Vcc) - 18.3A
	 *
	 * Vout = Vcc / 2 + i * (Vcc / 60A)
	 *
	 * SENSITIVITY = 1000 * 3300 / 55
	 *
	 * i = 60A * (Vout / Vcc) - 30A
	 */
#define ACS_SENSITIVITY 30
	bldc_filter_add_value(&g_current_filter, (1000 * 3300 / ACS_SENSITIVITY) * bldc->Vc/3300 - (1000 * 3300 / (ACS_SENSITIVITY * 2)));
	bldc->Icur = bldc_filter_get(&g_current_filter);
}


void bldc_load_sine(BLDC_TypeDef *bldc, TIM_TypeDef *TIMx, uint32_t throttle)
{
	uint32_t sine_driving_counter = bldc->counter + bldc->state * com_cycles_max;
	uint32_t sine_driving_state = SINE_SAMPLES * sine_driving_counter / (com_cycles_max * 6);

	uint32_t arr = LL_TIM_GetAutoReload(TIMx);
	uint32_t i1 = (sine_driving_state + SINE_SAMPLES * 0 / 3) % SINE_SAMPLES;
	uint32_t i2 = (sine_driving_state + SINE_SAMPLES * 2 / 3) % SINE_SAMPLES;
	uint32_t i3 = (sine_driving_state + SINE_SAMPLES * 1 / 3) % SINE_SAMPLES;
	uint32_t c1 = g_sine_buffer360[i1] * arr * throttle / 100 / 100; /* Divide by 100 for the sine %, divide by 100 for the throttle % */
	uint32_t c2 = g_sine_buffer360[i2] * arr * throttle / 100 / 100;
	uint32_t c3 = g_sine_buffer360[i3] * arr * throttle / 100 / 100;

	LL_TIM_OC_SetCompareCH1(TIMx, c1);
	LL_TIM_OC_SetCompareCH2(TIMx, c2);
	LL_TIM_OC_SetCompareCH3(TIMx, c3);
}

int32_t bldc_bootstrap(BLDC_TypeDef *bldc)
{
	if (bldc->bootstrap_stage == 0) {
		bldc->state = 0;
		bldc->counter = 0;
		LL_TIM_DisableAllOutputs(TIM_AMC);
		LL_TIM_CC_DisablePreload(TIM_AMC);
		LL_TIM_CC_EnableChannel(TIM_AMC,
								LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
								LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
								LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_OC_SetCompareCH1(TIM_AMC, 0);
		LL_TIM_OC_SetCompareCH2(TIM_AMC, 0);
		LL_TIM_OC_SetCompareCH3(TIM_AMC, 0);
		LL_TIM_EnableAllOutputs(TIM_AMC);
		++bldc->bootstrap_stage;
	} else {
		uint32_t bootstrap_throttle = (bldc->bootstrap_stage < 3) ? BOOTSTRAP_BASE_THROTTLE + bldc->bootstrap_stage : BOOTSTRAP_BASE_THROTTLE + bldc->bootstrap_stage / 2;
		uint32_t bootstrap_cycles = com_cycles_max / bldc->bootstrap_stage;

		bldc_load_sine(bldc, TIM_AMC, bootstrap_throttle);
		if (bldc->counter > bootstrap_cycles) {
			bldc->counter = 0;
			bldc->state = (bldc->state + 1) % 6;
			if (bldc->state == 0) {
				bldc->bootstrap_stage += 1;
				if (bldc->bootstrap_stage < BOOTSTRAP_STAGES / 2)
					bldc->bootstrap_stage += 1;
			}
		}
	}

	if (bldc->bootstrap_stage > BOOTSTRAP_STAGES)
		return 1;

	return 0;
}


void bldc_resistor_measurement_abc(BLDC_TypeDef *bldc)
{
	if (resmsr_counter == 0) {
		bldc_filter_reset(&g_resmsr_filter_0, 0);
		bldc_filter_reset(&g_resmsr_filter_1, 0);
		bldc_filter_reset(&g_resmsr_filter_2, 0);

		uint32_t arr = LL_TIM_GetAutoReload(TIM_AMC);
		LL_TIM_DisableAllOutputs(TIM_AMC);
		LL_TIM_CC_DisablePreload(TIM_AMC);
		LL_TIM_OC_SetCompareCH1(TIM_AMC, arr * MIN_THROTTLE_PERCENT / 100);
		LL_TIM_OC_SetCompareCH2(TIM_AMC, arr * MIN_THROTTLE_PERCENT / 100);
		LL_TIM_OC_SetCompareCH3(TIM_AMC, arr * MIN_THROTTLE_PERCENT / 100);

		LL_TIM_CC_EnableChannel(TIM_AMC, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
		LL_TIM_CC_EnableChannel(TIM_AMC, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
		LL_TIM_CC_EnableChannel(TIM_AMC, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_EnableAllOutputs(TIM_AMC);
	} else {
		bldc_filter_add_value(&g_resmsr_filter_0, injdata[0]);
		bldc_filter_add_value(&g_resmsr_filter_1, injdata[1]);
		bldc_filter_add_value(&g_resmsr_filter_2, injdata[2]);
		Vmsr0 = bldc_filter_get(&g_resmsr_filter_0);
		Vmsr1 = bldc_filter_get(&g_resmsr_filter_1);
		Vmsr2 = bldc_filter_get(&g_resmsr_filter_2);
	}
}

void bldc_adc_jeos_process()
{
	BLDC_TypeDef *bldc = &g_bldc;

	bldc->counter++;
	bldc_adc_data_acquisition(bldc);
	bldc_measure_bemf(bldc);

#ifdef DEBUG_SAMPLE_BUFFER
	g_sample_buffer[g_debug_counter].bemf = bldc->bemf;
	g_sample_buffer[g_debug_counter].state = bldc->state;
	g_sample_buffer[g_debug_counter].counter = bldc->counter;
	g_debug_counter = (g_debug_counter + 1) & SAMPLE_BUFFER_MASK;
#endif

	if (bldc->Icur > MAX_CURRENT_MILIAPMS) {
		bldc_set_error_substate(bldc, ERROR_CURRENT_FAULT);
		bldc_emergency_stop(bldc);
		return;
	}

	switch (bldc_get_substate(bldc)) {
	case SUBSTATE_ERROR:
		goto irrecoverable_error;
		return;

	case SUBSTATE_DISABLED:
		bldc_6step_stop(TIM_AMC);
		return;

	case SUBSTATE_RES_MEASUREMENT_INIT:
		resmsr_counter = 0;
		bldc_set_substate(bldc, SUBSTATE_RES_MEASUREMENT_ABC);
		break;

	case SUBSTATE_RES_MEASUREMENT_ABC:
		bldc_resistor_measurement_abc(bldc);
		if (resmsr_counter++ > RES_MEASUREMENT_CYCLES) {
			bldc_set_substate(bldc, SUBSTATE_DISABLED);
		}
		return;

//	case SUBSTATE_NONE:
	case SUBSTATE_MEASUREMENT1:
		if (bldc->Vh < 5 || bldc->Vl < 5 || bldc->Vb < 5) {
			bldc_set_error_substate(bldc, ERROR_MEASUREMENT);
//			BLDC_DEBUG_BKPT();
			goto irrecoverable_error;
			return;
		}
		bldc_set_substate(bldc, SUBSTATE_MEASUREMENT1);
		if (bldc->counter >= 2 && bldc->Vh > 1000) {
			bldc_voltage_measurement(bldc, &bldc->bemf_msr1);
			bldc_set_substate(bldc, SUBSTATE_MEASUREMENT2);
		}
		break;

	case SUBSTATE_MEASUREMENT2:
		if (bldc->Vh < 5 || bldc->Vl < 5 || bldc->Vb < 5) {
			bldc_set_error_substate(bldc, ERROR_MEASUREMENT);
//			BLDC_DEBUG_BKPT();
			goto irrecoverable_error;
			return;
		}

		/*
		 * Line equation:
		 * y = slope * x + intercept
		 */
		bldc_voltage_measurement(bldc, &bldc->bemf_msr2);
		bldc->bemf_mslope = (bldc->bemf_msr2.value - bldc->bemf_msr1.value) * 1000 / (bldc->bemf_msr2.counter - bldc->bemf_msr1.counter);
		bldc->bemf_intercept = (bldc->bemf_msr2.value * 1000 - bldc->bemf_mslope * bldc->bemf_msr2.counter) / 1000;

		if ((bldc->state % 2 == 0 && ((bldc->bemf_mslope <= 0 && bldc->bemf_intercept <= 0))) ||
			(bldc->state % 2 == 1 && ((bldc->bemf_mslope >= 0 && bldc->bemf_intercept >= 0)))) {
				bldc->bemf_msr1 = bldc->bemf_msr2;
				bldc_set_substate(bldc, SUBSTATE_MEASUREMENT2);
				break;
		}

		/*
		 * bldc->bemf_mslope can't be 0 here, because of the 'if statement' above.
		 */
		bldc->bemf_zerodetect = -bldc->bemf_intercept * 1000 / bldc->bemf_mslope;
		if (((int32_t)bldc->counter) > bldc->bemf_zerodetect) {
			bldc->zero_detected = bldc->bemf_zerodetect;
			bldc_set_substate(bldc, SUBSTATE_ZERODETECTED);
		} else {
			/*
			 * bldc->substate = SUBSTATE_MEASUREMENT2;
			 * so we keep refining the measurement of slope and intercept.
			 */
		}

		/*
		 * Error check
		 */
		if (bldc->throttle > 60 && bldc->counter > 250) {
			bldc_set_error_substate(bldc, ERROR_STATE_SPAN);
			return;
		}
		break;

	case SUBSTATE_ZERODETECTED:
		/*
		 * Error check
		 */
		bldc->last_zerodetected_time = jiffies;
		if (!bldc->bootstrap_zerodetected_time) {
			bldc->bootstrap_zerodetected_time = jiffies;
		}
		if (bldc->throttle > 70 && (bldc->zero_detected < 0 || bldc->zero_detected > 2000)) {
			bldc_set_error_substate(bldc, ERROR_INVALID_ZEROCROSSING);
			return;
		}
		break;

	case SUBSTATE_BOOTSTRAP_INIT:
		bldc->bootstrap_stage = 0;
		bldc_set_substate(bldc, SUBSTATE_BOOTSTRAP_PROGRESSING);
		return;

	case SUBSTATE_BOOTSTRAP_PROGRESSING:
		if (bldc_bootstrap(bldc))
			bldc_set_substate(bldc, SUBSTATE_BOOTSTRAP_COMPLETE);
		return;

	case SUBSTATE_BOOTSTRAP_COMPLETE:
		bldc->state = 5;
		bldc_6step_next(TIM_AMC, (bldc->state) % 6);
		LL_TIM_CC_EnablePreload(TIM_AMC);
		bldc_6step_next(TIM_AMC, (bldc->state + 1) % 6);
		bldc_set_substate(bldc, SUBSTATE_MEASUREMENT1);
		bldc->throttle = BOOTSTRAP_POST_THROTTLE;
		bldc->megathrottle_current = BOOTSTRAP_POST_THROTTLE * 1000000;
		bldc->megathrottle_time = jiffies;
		bldc_set_throttle(bldc, bldc->throttle);
		bldc->bootstrap_time = jiffies;
		bldc->bootstrap_zerodetected_time = 0ULL;
		bldc->rotation_hz = 30;
		LL_TIM_EnableAllOutputs(TIM_AMC);
#ifdef TEST_BOOTSTRAP
		LL_TIM_DisableAllOutputs(TIM_AMC);
		bldc_6step_stop(TIM_AMC);
#endif
		return;
	}

	if (bldc_detect_bemf(bldc, bldc->Vh, bldc->Vb, bldc->Vl) > 0) {
		bldc_generate_com_event(&g_bldc);
		/*
		 * COM event has been generated, so start the measurement
		 * cycle all over again
		 */
		bldc_set_substate(bldc, SUBSTATE_MEASUREMENT1);
		return;
	}

	if (bldc->counter > com_cycles_max / 10) {
		if (--bldc->bootstrap_counter <= 0) {
			bldc->substate = SUBSTATE_BOOTSTRAP_INIT;
		} else {
			bldc_6step_next(TIM_AMC, (bldc->state + 1) % 6);
			bldc_generate_com_event(&g_bldc);
			return;
		}
	}

	return;

irrecoverable_error:
	bldc_6step_stop(TIM_AMC);
	bldc->output_counter++;
	memcpy(&g_bldc_saved, bldc, sizeof(BLDC_TypeDef));
	return;

}


void bldc_adc_jeos_callback()
{
	g_bldc.jeos_begin = LL_TIM_GetCounter(TIM_AMC);
	bldc_adc_jeos_process();
	g_bldc.jeos_end = LL_TIM_GetCounter(TIM_AMC);
	g_bldc.jeos_time = g_bldc.jeos_end - g_bldc.jeos_begin;
}

void bldc_print_info(BLDC_TypeDef *bldc)
{
	printf("< %16llu > %1lu: slp: %7ld, incpt: %5ld, zd: %3ld, ibemf: %5ld, %5lu Hz, zd: %3ld (%3ld), sp: %7.2f (%3lu), "
			" Cur: %5ld, (%4ld, %5ld) - (%4ld, %5ld), JEOS: (%4lu - %4lu) / %4lu\n",
			jiffies,
			bldc->state,
			bldc->bemf_mslope,
			bldc->bemf_intercept,
			bldc->bemf_zerodetect,
			bldc->integral_bemf,
			bldc->rotation_hz,
			bldc->zero_detected,
			bldc->counter,
			bldc->megathrottle_current / 1000000.0,
			bldc->throttle,
			bldc->Icur,
			bldc->bemf_msr1.counter, bldc->bemf_msr1.value,
			bldc->bemf_msr2.counter, bldc->bemf_msr2.value,
			bldc->jeos_begin,
			bldc->jeos_end,
			amc_autoreload);

}

int main(int argc, char* argv[])
{
	uint32_t last_print = 0;
	uint32_t i = 0;

	/*
	 * Init BLDC context
	 */
	memset(&g_bldc, 0, sizeof(g_bldc));

	/*
	 * Init current filter
	 */
	bldc_filter_init(&g_current_filter, 995);

	/*
	 * Init resistor measurement filter
	 */
	bldc_filter_init(&g_resmsr_filter_0, 750);
	bldc_filter_init(&g_resmsr_filter_1, 750);
	bldc_filter_init(&g_resmsr_filter_2, 750);

	bldc_init_sine();

	bldc_interrupts_config();
	bldc_gpio_config();
	bldc_interrupts_enable();

	/*
	 * Init DMA for USART tx
	 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	bldc_dma_init(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE);

	/*
	 * Init USART1
	 */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	bldc_uart_init(USART1, UART_BAUD_RATE, LL_USART_HWCONTROL_NONE);
	bldc_uart_start(USART1);

	printf("bldc-cc-f4!\n");
	printf("System clock: %lu Hz\n", SystemCoreClock);

	/*
	 * Init ADC1 for BEMF reading
	 */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	bldc_adc_init(ADC1);
	bldc_adc_start(ADC1);

	/*
	 * Init periodic timer for blinking led
	 */
	LL_APB1_GRP1_EnableClock(TIM_BLINK_PERIPH);
	bldc_timer_init(TIM_BLINK, 5, SystemCoreClock, 0xFFFF);
	LL_TIM_EnableIT_UPDATE(TIM_BLINK);
	bldc_timer_start(TIM_BLINK);

	/*
	 * Init jiffies timer
	 */
	LL_APB1_GRP1_EnableClock(TIM_JIFFIES_PERIPH);
	bldc_timer_init(TIM_JIFFIES, JIFFIES_HZ, SystemCoreClock, 0xFFFF);
	LL_TIM_EnableIT_UPDATE(TIM_JIFFIES);
	bldc_timer_start(TIM_JIFFIES);


	/*
	 * Init PWM decoder
	 */
	LL_APB1_GRP1_EnableClock(TIM_PWM_PERIPH);
	bldc_timer_init_pwm_decoder(TIM_PWM, 40, SystemCoreClock, 0xFFFF);

	/*
	 * Init ADC trigger
	 * The ADC trigger timer is the one that triggers the BEMF measurement after it
	 * introduces a small delay. The nanoseconds delay is configured with BEMF_MEASURE_DELAY_NS.
	 */
	LL_APB1_GRP1_EnableClock(TIM_TRG_PERIPH);
	bldc_adc_trigger_init(TIM_TRG, 1000000000UL / BEMF_MEASURE_DELAY_NS, SystemCoreClock, 0xFFFFFFFF, LL_TIM_OCMODE_PWM1);

	/*
	 * Init 6 step driving Timer
	 */
	LL_APB2_GRP1_EnableClock(TIM_AMC_PERIPH);
	bldc_6step_init(TIM_AMC, SWITCHING_FREQUENCY, SystemCoreClock, 0xFFFF, LL_TIM_OCMODE_PWM1,
			LL_TIM_OCPOLARITY_HIGH, LL_TIM_OCPOLARITY_LOW, DEAD_TIME_NS, REPETIONS_TO_UPDATE);


	amc_autoreload = LL_TIM_GetAutoReload(TIM_AMC);
	amc_switching_hz = bldc_6step_switching_frequency(TIM_AMC, SystemCoreClock);
	pwm_clock_hz = bldc_timer_get_clock_frequency(TIM_PWM, SystemCoreClock);
	com_cycles_max = 1000 * amc_switching_hz / (MIN_MILLIHERTZ * SINE_STATES * MECHANICAL_DEGREES_RATIO);

	bldc_adc_trigger_start(TIM_TRG);

	bldc_set_substate(&g_bldc, SUBSTATE_RES_MEASUREMENT_INIT);
	bldc_6step_start(TIM_AMC);
	while (bldc_get_substate(&g_bldc) == SUBSTATE_RES_MEASUREMENT_INIT || bldc_get_substate(&g_bldc) == SUBSTATE_RES_MEASUREMENT_ABC) {
		HAL_Delay(500);
		printf("RES 0: %4ld, RES 1: %4ld, RES 2: %4ld, resmsr_counter: %ld, OC: %ld, substate: %ld\n",
				Vmsr0, Vmsr1, Vmsr2, resmsr_counter, LL_TIM_OC_GetCompareCH1(TIM_AMC), g_bldc.substate);
	}
	printf("RES 0: %4ld, RES 1: %4ld, RES 2: %4ld, resmsr_counter: %ld, OC: %ld, substate: %ld\n",
			Vmsr0, Vmsr1, Vmsr2, resmsr_counter, LL_TIM_OC_GetCompareCH1(TIM_AMC), g_bldc.substate);

	printf("Sine120: \n");
	for (i = 0; i < SINE_SAMPLES; i++) {
		printf("%ld,", g_sine_buffer120[i]);
	}
	printf("\n");

	printf("Sine180: \n");
	for (i = 0; i < SINE_SAMPLES; i++) {
		printf("%ld,", g_sine_buffer360[i]);
	}
	printf("\n");

	printf("PWM TIM clock %lu Hz\n", bldc_timer_get_clock_frequency(TIM_PWM, SystemCoreClock));
	printf("AMC TIM clock %lu Hz, ARR: %lu\n", bldc_timer_get_clock_frequency(TIM_AMC, SystemCoreClock), LL_TIM_GetAutoReload(TIM_AMC));
	printf("com_cycles_max: %ld\n", com_cycles_max);

	/*
	 * Start the timers
	 */
	bldc_timer_pwm_decoder_start(TIM_PWM);

	while(1) {
		uint32_t output_counter;
		bldc_gpio_activelow_write(LL_PORT(LED_STATUS_PIN), LL_PINMASK(LED_STATUS_PIN), LL_TIM_IsEnabledCounter(TIM_AMC), LED_STATUS_ACTIVE_LOW);

		__disable_irq();
		output_counter = g_bldc_saved.output_counter;
		__enable_irq();

#ifdef DEBUG_PRINT
	if (last_print != output_counter) {
		BLDC_TypeDef bldc_local;
		__disable_irq();
		memcpy(&bldc_local, &g_bldc_saved, sizeof(g_bldc_saved));
		__enable_irq();
//		last_print = g_bldc_saved.output_counter;

		bldc_print_info(&bldc_local);
		if (g_bldc_saved.substate == SUBSTATE_ERROR) {
			printf("\n<------ ERROR:  %lu\n", g_bldc_saved.error);
			if (g_bldc.error == ERROR_MEASUREMENT) {
				printf("Vh = %ld, Vl = %ld, Vb = %ld, counter: %lu, state: %lu\n",
						g_bldc_saved.Vh, g_bldc_saved.Vl, g_bldc_saved.Vb, g_bldc_saved.counter, g_bldc.state);
			}
#ifdef DEBUG_SAMPLE_BUFFER
			for (i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
				printf("<%lu::%3lu> %5ld\n",
						g_sample_buffer[g_debug_counter].state,
						g_sample_buffer[g_debug_counter].counter,
						g_sample_buffer[g_debug_counter].bemf);
				g_debug_counter = (g_debug_counter + 1) & SAMPLE_BUFFER_MASK;
			}
#endif
		}
		last_print = bldc_local.output_counter;
	}
#endif

	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
