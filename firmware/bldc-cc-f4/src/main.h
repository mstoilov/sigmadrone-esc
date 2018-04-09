#ifndef _MAIN_H_
#define _MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bldc-filter.h"

#define SUBSTATE_DISABLED				0
#define SUBSTATE_RES_MEASUREMENT_INIT	1
#define SUBSTATE_RES_MEASUREMENT_ABC	2
#define SUBSTATE_RES_MEASUREMENT_END	3
//#define SUBSTATE_NONE					4
#define SUBSTATE_MEASUREMENT1			6
#define SUBSTATE_MEASUREMENT2 			7
#define SUBSTATE_ZERODETECTED			8
#define SUBSTATE_BOOTSTRAP_INIT			9
#define SUBSTATE_BOOTSTRAP_PROGRESSING	10
#define SUBSTATE_BOOTSTRAP_COMPLETE		11

//#define SUBSTATE_BOOTSTRAPPED			20
#define SUBSTATE_ERROR					21

#define ERROR_NONE						0
#define ERROR_MEASUREMENT				1
#define ERROR_CURRENT_FAULT				2
#define ERROR_STATE_SPAN				3
#define ERROR_INVALID_ZEROCROSSING		4

#if defined(DEBUG)
#define BLDC_DEBUG_BKPT()  asm volatile ("bkpt 0")
#else
#define BLDC_DEBUG_BKPT()  do {} while (0)
#endif


typedef struct BLDC_Measurement_ {
	int32_t value;
	int32_t counter;
	int32_t Vh;
	int32_t Vb;
	int32_t Vl;
} BLDC_Measurement;

typedef struct BLDC_TypeDef_ {
	uint32_t state;
	uint32_t substate;
	uint32_t counter;
	uint32_t rotation_hz;
	uint32_t last_counter;
	int32_t zero_detected;
	int32_t integral_bemf;
	int32_t bemf_mslope;
	int32_t bemf_intercept;
	int32_t bemf_zerodetect;
	int32_t bemf;
	BLDC_Measurement bemf_msr1;
	BLDC_Measurement bemf_msr2;
	int32_t throttle;						/* Desired throttle value */
	float throttle_speed;
	int32_t megathrottle_current;			/* Current throttle value multiplied by 1000000 */
	uint64_t megathrottle_time;				/* The time the megathrottle value was pushed to the hardaware */
	uint32_t output_counter;
	int32_t bootstrap_counter;
	uint64_t bootstrap_time;
	uint64_t bootstrap_zerodetected_time;
	uint64_t last_zerodetected_time;
	int32_t bootstrap_stage_counter;
	int32_t bootstrap_stage;
	uint32_t com_counter;
	uint32_t error_counter;
	uint32_t error;
	uint32_t compdata[4];
	int32_t Vl;
	int32_t Vh;
	int32_t Vb;
	int32_t Vc;
	int32_t Icur;
	int32_t jeos_time;
	int32_t jeos_begin;
	int32_t jeos_end;
} BLDC_TypeDef;

void bldc_pwm_cc_callback();
void bldc_timer_blink_callback();
void bldc_timer_jiffies_callback();
void bldc_adc_eocs_callback();
void bldc_adc_jeos_callback();
void bldc_adc_eocs_callback();
void bldc_user_button_callback();
void bldc_current_fault_callback();
void bldc_generate_com_event(BLDC_TypeDef *bldc);

#ifdef __cplusplus
}
#endif

#endif
