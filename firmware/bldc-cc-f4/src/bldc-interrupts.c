
#include "bldc-include.h"
#include "bldc-config.h"
#include "bldc-interrupts.h"
#include "bldc-gpio.h"
#include "main.h"

void bldc_interrupts_config()
{
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);

	/*
	 * Configure EXTI
	 */
	__SYSCFG_CLK_ENABLE();
	LL_SYSCFG_SetEXTISource(LL_PORTNUM(BTN_USER_PIN), LL_EXTI_LINE(BTN_USER_PIN));
	LL_EXTI_EnableIT_0_31(LL_PINMASK(BTN_USER_PIN));
	LL_EXTI_EnableFallingTrig_0_31(LL_PINMASK(BTN_USER_PIN));
	NVIC_SetPriority(BTN_USER_IRQ, BTN_USER_IRQ_PRIORITY);


	LL_SYSCFG_SetEXTISource(LL_PORTNUM(CURRENT_FAULT_PIN), LL_EXTI_LINE(CURRENT_FAULT_PIN));
	LL_EXTI_EnableIT_0_31(LL_PINMASK(CURRENT_FAULT_PIN));
	LL_EXTI_EnableFallingTrig_0_31(LL_PINMASK(CURRENT_FAULT_PIN));

	/*
	 * USR BUTTON and CURRENT FAULT share the same IRQ
	 */
	NVIC_SetPriority(BTN_USER_IRQ, BTN_USER_IRQ_PRIORITY);

	/*
	 * Configure TIM3 interrupt
	 */
	NVIC_SetPriority(TIM3_IRQn, 8);

	/*
	 * Configure TIM4 interrupt
	 */
	NVIC_SetPriority(TIM4_IRQn, 8);

	/*
	 * Configure TIM5 interrupt
	 */
	NVIC_SetPriority(TIM5_IRQn, 4);

	/*
	 * Configure ADC1 interrupt
	 */
	NVIC_SetPriority(ADC_IRQn, 2);

	/*
	 * Configure USART1 interrupt
	 */
	NVIC_SetPriority(USART1_IRQn, 12);

	/*
	 * Configure DMA2 Stream7, used by the USART1 tx
	 */
	NVIC_SetPriority(DMA2_Stream7_IRQn, 0);

}

void bldc_interrupts_enable()
{
	NVIC_EnableIRQ(BTN_USER_IRQ);
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_EnableIRQ(TIM5_IRQn);
	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

void bldc_interrupts_disable()
{
	NVIC_DisableIRQ(BTN_USER_IRQ);
	NVIC_DisableIRQ(TIM3_IRQn);
	NVIC_DisableIRQ(TIM4_IRQn);
	NVIC_DisableIRQ(TIM5_IRQn);
	NVIC_DisableIRQ(ADC_IRQn);
	NVIC_DisableIRQ(USART1_IRQn);
	NVIC_DisableIRQ(DMA2_Stream7_IRQn);
}


void SysTick_Handler (void)
{
#if defined(USE_HAL_DRIVER)
	HAL_IncTick();
#endif
}

#if _ESC_MODEL_SIGMASERVO_

void EXTI2_IRQHandler(void)
{
	if (LL_EXTI_IsActiveFlag_0_31(LL_PINMASK(CURRENT_FAULT_PIN)) != RESET) {
		LL_EXTI_ClearFlag_0_31(LL_PINMASK(CURRENT_FAULT_PIN));
		bldc_current_fault_callback();
	}
}


void EXTI4_IRQHandler(void)
{
	if (LL_EXTI_IsActiveFlag_0_31(LL_PINMASK(BTN_USER_PIN)) != RESET) {
		LL_EXTI_ClearFlag_0_31(LL_PINMASK(BTN_USER_PIN));
		bldc_user_button_callback();
	}
}
#else

void EXTI15_10_IRQHandler(void)
{
	if (LL_EXTI_IsActiveFlag_0_31(LL_PINMASK(BTN_USER_PIN)) != RESET) {
		LL_EXTI_ClearFlag_0_31(LL_PINMASK(BTN_USER_PIN));
		bldc_user_button_callback();
	}

	if (LL_EXTI_IsActiveFlag_0_31(LL_PINMASK(CURRENT_FAULT_PIN)) != RESET) {
		LL_EXTI_ClearFlag_0_31(LL_PINMASK(CURRENT_FAULT_PIN));
		bldc_current_fault_callback();
	}

}


#endif

void TIM3_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_CC1(TIM3)) {
		LL_TIM_ClearFlag_CC1(TIM3);
		bldc_pwm_cc_callback();
	}

	WRITE_REG(TIM3->SR, 0);
}

void TIM4_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM4)) {
		LL_TIM_ClearFlag_UPDATE(TIM4);
		bldc_timer_blink_callback();
	}

	WRITE_REG(TIM4->SR, 0);
}

void TIM5_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM5)) {
		LL_TIM_ClearFlag_UPDATE(TIM5);
		bldc_timer_jiffies_callback();
	}

	WRITE_REG(TIM5->SR, 0);
}


void ADC_IRQHandler(void)
{
	if (LL_ADC_IsActiveFlag_AWD1(ADC1) != 0) {
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_AWD1(ADC1);
	}

	if (LL_ADC_IsActiveFlag_EOCS(ADC1) != 0) {
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_EOCS(ADC1);
		bldc_adc_eocs_callback();
	}

	if (LL_ADC_IsActiveFlag_JEOS(ADC1) != 0) {
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_JEOS(ADC1);
		bldc_adc_jeos_callback();
	}

	if (LL_ADC_IsActiveFlag_OVR(ADC1) != 0) {
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_OVR(ADC1);
	}
}

void USART1_IRQHandler(void)
{

}

void __attribute__ ((weak, alias ("Default_Handler")))
DMA2_Stream7_TC_callback(void)
{

}


void DMA2_Stream7_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_DME7(DMA2)) {
		LL_DMA_ClearFlag_DME7(DMA2);

	}
	if (LL_DMA_IsActiveFlag_FE7(DMA2)) {
		LL_DMA_ClearFlag_FE7(DMA2);

	}
	if (LL_DMA_IsActiveFlag_HT7(DMA2)) {
		LL_DMA_ClearFlag_HT7(DMA2);

	}
	if (LL_DMA_IsActiveFlag_TC7(DMA2)) {
		LL_DMA_ClearFlag_TC7(DMA2);
		DMA2_Stream7_TC_callback();
	}
	if (LL_DMA_IsActiveFlag_TE7(DMA2)) {
		LL_DMA_ClearFlag_TE7(DMA2);

	}

}

