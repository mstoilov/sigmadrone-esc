#include "bldc-include.h"
#include "pinnames.h"

#define ACTIVE_LOW_TRUE				(1)
#define ACTIVE_LOW_FALSE			(0)

#define TIM_AMC						TIM1
#define TIM_AMC_PERIPH				LL_APB2_GRP1_PERIPH_TIM1
#define TIM_TRG						TIM2
#define TIM_TRG_PERIPH				LL_APB1_GRP1_PERIPH_TIM2
#define TIM_PWM						TIM3
#define TIM_PWM_PERIPH				LL_APB1_GRP1_PERIPH_TIM3
#define TIM_BLINK					TIM4
#define TIM_BLINK_PERIPH			LL_APB1_GRP1_PERIPH_TIM4
#define TIM_JIFFIES					TIM5
#define TIM_JIFFIES_PERIPH			LL_APB1_GRP1_PERIPH_TIM5
#define JIFFIES_HZ					1000000


#define PWM_MODE					LL_TIM_OCMODE_PWM1
#define DEAD_TIME_NS				320
#define ADC_SAMPLING_CYCLES			LL_ADC_SAMPLINGTIME_28CYCLES

#define _ESC_MODEL_SIGMASERVO_ 1

#if _ESC_MODEL_SIGMASERVO_


#define USART_TX					PA_15
//#define USART_RX					PB_3
#define CURRENT_FAULT_PIN			PA_2

#define PWM_AL						PB_13
#define PWM_BL						PB_14
#define PWM_CL						PB_15

#define PIN_ADC1					PC_0
#define PIN_ADC2					PC_1
#define PIN_ADC3					PC_2
#define PIN_ADC4					PA_0
#define PIN_ADC5					PA_1

#define CHANNEL_PHASE_A				LL_ADC_CHANNEL_10
#define CHANNEL_PHASE_B				LL_ADC_CHANNEL_11
#define CHANNEL_PHASE_C				LL_ADC_CHANNEL_12
#define CHANNEL_CURRENT_ADC			LL_ADC_CHANNEL_0

#define ADC_A						0
#define ADC_B						1
#define ADC_C						2

#define LED_STATUS_PIN				PA_6
#define LED_WARNING_PIN				PA_5

#define PWM_AH						PA_8
#define PWM_BH						PA_9
#define PWM_CH						PA_10

#define LED_STATUS_ACTIVE_LOW		ACTIVE_LOW_TRUE

#define BTN_USER_PIN				PA_4
#define BTN_USER_IRQ				EXTI4_IRQn
#define BTN_USER_IRQ_PRIORITY		12

#define PWM_INPUT_PIN				PB_4
#define PWM_INPUT_AF				LL_GPIO_AF_2


#else

#if _ESC_MODEL_ZEKENG_

#define USART_TX					PA_15
#define USART_RX					PB_3
#define CURRENT_FAULT_PIN			PB_13

#else

#define USART_TX					PB_6
#define USART_RX					PB_7
#define CURRENT_FAULT_PIN			PA_15

#endif

#define PWM_AL						PA_7
#define PWM_BL						PB_0
#define PWM_CL						PB_1

#define PIN_ADC1					PC_1
#define PIN_ADC2					PC_2
#define PIN_ADC3					PC_3
#define PIN_ADC4					PC_0
#define PIN_ADC5					PA_0

#define CHANNEL_PHASE_A				LL_ADC_CHANNEL_13
#define CHANNEL_PHASE_B				LL_ADC_CHANNEL_12
#define CHANNEL_PHASE_C				LL_ADC_CHANNEL_11
#define CHANNEL_CENTER				LL_ADC_CHANNEL_10
#define CHANNEL_CURRENT_ADC			LL_ADC_CHANNEL_0

#define ADC_A						2
#define ADC_B						1
#define ADC_C						0

#define LED_STATUS_PIN				PA_11
#define LED_WARNING_PIN				PA_12

#define PWM_AH						PA_8
#define PWM_BH						PA_9
#define PWM_CH						PA_10

#define LED_STATUS_ACTIVE_LOW		ACTIVE_LOW_TRUE

#define BTN_USER_PIN				PC_13
#define BTN_USER_IRQ				EXTI15_10_IRQn
#define BTN_USER_IRQ_PRIORITY		12

#define PWM_INPUT_PIN				PB_4
#define PWM_INPUT_AF				LL_GPIO_AF_2

#endif


#define UART_BAUD_RATE				230400
#define VOLTAGE_MCU_POWER			3300		/* Milli-Volts */
#define BEMF_SENSE_DEVIDER			(47000/4700)
#define RES_MEASUREMENT_CYCLES		1000
#define MAX_CURRENT_MILIAPMS		30000
#define	SANITY_CHECK_VOLTAGE_RAIL	8000		/* Milli-Volts */
#define	SANITY_CHECK_VOLTAGE_BEMF	1000		/* Milli-Volts */
#define BEMF_MAX_KICKBACK_PERIOD	3
#define SINE_STATES					6
#define MECHANICAL_DEGREES_RATIO	8
#define REPETIONS_TO_UPDATE			1			/* MIN valeue: 1 */
#define MIN_MILLIHERTZ				1000
#define BOOTSTRAP_BASE_THROTTLE		10
#define BOOTSTRAP_POST_THROTTLE		40
#define MIN_THROTTLE_PERCENT		51
#define BOOTSTRAP_COUNTER_INIT		20
#define SWITCHING_FREQUENCY			100000
#define BOOTSTRAP_STAGES			30
#define BEMF_INTEGRAL_THRESHOLD		3300
#define BEMF_MEASURE_DELAY_NS		550UL
#define THROTTLE_MIN_PWM_DUTY		1100	/* Millisceonds */
#define THROTTLE_SPEED_HZ_250		400		/* Throttle speed when the throttle is at 250 Hz */
#define THROTTLE_SPEED_HZ_0			200		/* Throttle speed when the throttle is at 0 Hz */
#define START_WITH_THROTTLE
#define COMPENSATE_DATA_BIAS
//#define TEST_BOOTSTRAP

