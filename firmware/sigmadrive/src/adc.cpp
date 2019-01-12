#include <algorithm>
#include "stm32f4xx_ll_bus.h"
#include "adc.h"
#include "interruptmanager.h"

Adc::Adc(ADC_TypeDef* ADCx,
		uint32_t resolution,
		uint32_t samplingTime,
		uint32_t injectedTrigger,
		const std::vector<GPIOPin>& pins,
		const std::vector<uint32_t>& injChannels)
	: ADCx_(ADCx)
	, resolution_(resolution)
	, injdataSize_(injChannels.size())
{
	if (ADCx_ == ADC1)
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	for (auto& pin : pins)
		pin.Init();

	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADCx_), LL_ADC_CLOCK_SYNC_PCLK_DIV2);

	InterruptManager::instance().Callback_EnableIRQ(ADC_IRQn, 0, &Adc::IRQHandler, this);

	InitInjectedChannels(resolution, samplingTime, injChannels);
	SetInjectedTrigger(injectedTrigger);
}

Adc::~Adc()
{

}

void Adc::SetInjectedTrigger(uint32_t trigger)
{
	if (trigger == LL_ADC_INJ_TRIG_SOFTWARE) {
	    LL_ADC_INJ_SetTriggerSource(ADCx_, LL_ADC_INJ_TRIG_SOFTWARE);
	} else {
	    LL_ADC_INJ_SetTriggerSource(ADCx_, trigger);
	    LL_ADC_INJ_StartConversionExtTrig(ADCx_, LL_ADC_INJ_TRIG_EXT_FALLING);
	    LL_ADC_INJ_SetTrigAuto(ADCx_, LL_ADC_INJ_TRIG_INDEPENDENT);
	}
}


void Adc::InitInjectedChannels(uint32_t resolution, uint32_t samplingTime, const std::vector<uint32_t>& injChannels)
{
	LL_ADC_SetResolution(ADCx_, resolution);

	/* Set ADC conversion data alignment */
	LL_ADC_SetResolution(ADCx_, LL_ADC_DATA_ALIGN_RIGHT);

    /* Set Set ADC sequencers scan mode, for all ADC groups                   */
    /* (group regular, group injected).                                       */
    LL_ADC_SetSequencersScanMode(ADCx_, LL_ADC_SEQ_SCAN_ENABLE);

    /* Set ADC channels sampling time */
    for (auto ch : injChannels) {
        LL_ADC_SetChannelSamplingTime(ADCx_, ch, samplingTime);
    }

	/* Set ADC group injected sequencer discontinuous mode */
	LL_ADC_INJ_SetSequencerDiscont(ADCx_, LL_ADC_INJ_SEQ_DISCONT_DISABLE);

	switch (injChannels.size()) {
	default:
	case 1:
		LL_ADC_INJ_SetSequencerLength(ADCx_, LL_ADC_INJ_SEQ_SCAN_DISABLE);
		break;
	case 2:
		LL_ADC_INJ_SetSequencerLength(ADCx_, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS);
		break;
	case 3:
		LL_ADC_INJ_SetSequencerLength(ADCx_, LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS);
		break;
	case 4:
		LL_ADC_INJ_SetSequencerLength(ADCx_, LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS);
		break;
	};

	/* Set ADC group injected sequencer length and scan direction */
	if (injChannels.size() >= 1) {
		LL_ADC_INJ_SetSequencerRanks(ADCx_, LL_ADC_INJ_RANK_1, injChannels[0]);
	}
	if (injChannels.size() >= 2) {
		LL_ADC_INJ_SetSequencerRanks(ADCx_, LL_ADC_INJ_RANK_2, injChannels[1]);
	}
	if (injChannels.size() >= 3) {
		LL_ADC_INJ_SetSequencerRanks(ADCx_, LL_ADC_INJ_RANK_3, injChannels[2]);
	}
	if (injChannels.size() >= 4) {
		LL_ADC_INJ_SetSequencerRanks(ADCx_, LL_ADC_INJ_RANK_4, injChannels[3]);
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
	if (LL_ADC_IsActiveFlag_JEOS(ADCx_) != 0) {
		LL_ADC_ClearFlag_JEOS(ADCx_);
		handle_jeos();
	}

	if (LL_ADC_IsActiveFlag_AWD1(ADCx_) != 0) {
		LL_ADC_ClearFlag_AWD1(ADCx_);
		handle_awd1();
	}

	if (LL_ADC_IsActiveFlag_EOCS(ADCx_) != 0) {
		LL_ADC_ClearFlag_EOCS(ADCx_);
		handle_eocs();
	}

	if (LL_ADC_IsActiveFlag_OVR(ADCx_) != 0) {
		/* Clear flag ADC group regular overrun */
		LL_ADC_ClearFlag_OVR(ADCx_);
		handle_overrun();
	}

}

void Adc::handle_jeos()
{
	__disable_irq();
	injdata_[0] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_1), resolution_);
	injdata_[1] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_2), resolution_);
	injdata_[2] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_3), resolution_);
	injdata_[3] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_4), resolution_);
	__enable_irq();

	callback_JEOS_(injdata_, injdataSize_);
}

void Adc::handle_overrun()
{

}

void Adc::handle_eocs()
{

}

void Adc::handle_awd1()
{

}

