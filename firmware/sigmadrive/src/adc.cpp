#include "stm32f4xx_ll_bus.h"
#include "adc.h"
#include "interruptmanager.h"

Adc::Adc(ADC_TypeDef* ADCx, const std::vector<GPIOPin>& pins)
	:ADCx_(ADCx)
{
	if (ADCx_ == ADC1)
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	for (auto& pin : pins)
		pin.Init();

	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADCx_), LL_ADC_CLOCK_SYNC_PCLK_DIV2);

	NVIC_SetPriority(ADC_IRQn, 0);
	NVIC_EnableIRQ(ADC_IRQn);
	InterruptManager::instance().Callback(ADC_IRQn, &Adc::IRQHandler, this);

	InitCurrentFeedback();
}

Adc::~Adc()
{

}


#define CURRENT_FB_A	LL_ADC_CHANNEL_13
#define CURRENT_FB_B	LL_ADC_CHANNEL_14
#define CURRENT_FB_C	LL_ADC_CHANNEL_15
#define CURRENT_FB_R	LL_ADC_CHANNEL_4

void Adc::InitCurrentFeedback()
{
	uint32_t SamplingTime = LL_ADC_SAMPLINGTIME_3CYCLES;

	LL_ADC_SetResolution(ADCx_, LL_ADC_RESOLUTION_12B);

	/* Set ADC conversion data alignment */
	LL_ADC_SetResolution(ADCx_, LL_ADC_DATA_ALIGN_RIGHT);

	/* Set Set ADC sequencers scan mode, for all ADC groups                   */
	/* (group regular, group injected).                                       */
//		LL_ADC_SetSequencersScanMode(ADCx_, LL_ADC_SEQ_SCAN_DISABLE);

    /* Set Set ADC sequencers scan mode, for all ADC groups                   */
    /* (group regular, group injected).                                       */
    LL_ADC_SetSequencersScanMode(ADCx_, LL_ADC_SEQ_SCAN_ENABLE);

    /* Set ADC channels sampling time */
    LL_ADC_SetChannelSamplingTime(ADCx_, CURRENT_FB_A, SamplingTime);
    LL_ADC_SetChannelSamplingTime(ADCx_, CURRENT_FB_B, SamplingTime);
    LL_ADC_SetChannelSamplingTime(ADCx_, CURRENT_FB_C, SamplingTime);
    LL_ADC_SetChannelSamplingTime(ADCx_, CURRENT_FB_R, SamplingTime);

    LL_ADC_INJ_SetTriggerSource(ADCx_, LL_ADC_INJ_TRIG_EXT_TIM2_CH1);
//    LL_ADC_INJ_SetTriggerSource(ADCx_, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);
    LL_ADC_INJ_StartConversionExtTrig(ADCx_, LL_ADC_INJ_TRIG_EXT_FALLING);
    LL_ADC_INJ_SetTrigAuto(ADCx_, LL_ADC_INJ_TRIG_INDEPENDENT);

	/* Set ADC group injected sequencer length and scan direction */
	LL_ADC_INJ_SetSequencerLength(ADCx_, LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS);

	/* Set ADC group injected sequencer discontinuous mode */
	LL_ADC_INJ_SetSequencerDiscont(ADCx_, LL_ADC_INJ_SEQ_DISCONT_DISABLE);

	if (LL_ADC_INJ_GetSequencerLength(ADCx_) == LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS) {
	/* Set ADC group injected sequence: channel on the selected sequence rank. */
		LL_ADC_INJ_SetSequencerRanks(ADCx_, LL_ADC_INJ_RANK_1, CURRENT_FB_A);
		LL_ADC_INJ_SetSequencerRanks(ADCx_, LL_ADC_INJ_RANK_2, CURRENT_FB_B);
		LL_ADC_INJ_SetSequencerRanks(ADCx_, LL_ADC_INJ_RANK_3, CURRENT_FB_C);
		LL_ADC_INJ_SetSequencerRanks(ADCx_, LL_ADC_INJ_RANK_4, CURRENT_FB_R);
	} else if (LL_ADC_INJ_GetSequencerLength(ADCx_) == LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS) {
		LL_ADC_INJ_SetSequencerRanks(ADCx_, LL_ADC_INJ_RANK_2, CURRENT_FB_A);
		LL_ADC_INJ_SetSequencerRanks(ADCx_, LL_ADC_INJ_RANK_3, CURRENT_FB_B);
		LL_ADC_INJ_SetSequencerRanks(ADCx_, LL_ADC_INJ_RANK_4, CURRENT_FB_C);
	}

	LL_ADC_EnableIT_JEOS(ADCx_);
}

void Adc::Start()
{
	LL_ADC_Enable(ADCx_);
}

void Adc::Stop()
{
	LL_ADC_Disable(ADCx_);
}

void Adc::IRQHandler(void)
{
	if (LL_ADC_IsActiveFlag_AWD1(ADCx_) != 0) {
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_AWD1(ADCx_);
		handle_awd1();
	}

	if (LL_ADC_IsActiveFlag_EOCS(ADCx_) != 0) {
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_EOCS(ADCx_);
		handle_eocs();
	}

	if (LL_ADC_IsActiveFlag_JEOS(ADCx_) != 0) {
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_JEOS(ADCx_);
		handle_jeos();
	}

	if (LL_ADC_IsActiveFlag_OVR(ADCx_) != 0) {
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_OVR(ADCx_);
		handle_overrun();
	}

}

void Adc::handle_overrun()
{

}

void Adc::handle_eocs()
{

}

void Adc::handle_jeos()
{
	__disable_irq();
	injdata_[0] = 3300/2 - __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_1), LL_ADC_RESOLUTION_12B);
	injdata_[1] = 3300/2 - __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_2), LL_ADC_RESOLUTION_12B);
	injdata_[2] = 3300/2 - __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_3), LL_ADC_RESOLUTION_12B);
	injdata_[3] = 3300/2 - __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_4), LL_ADC_RESOLUTION_12B);

	__enable_irq();

}

void Adc::handle_awd1()
{

}

