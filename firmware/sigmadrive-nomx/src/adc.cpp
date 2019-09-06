#include <algorithm>
#include "adc.h"
#include "interruptmanager.h"

unsigned int Adc::instance_count_ = 0UL;

/*
 * Regular channels use DMA transfer
 */
Adc::Adc(const std::vector<GPIOPin>& pins,
		const std::vector<uint32_t>& regChannels,
		const std::vector<uint32_t>& injChannels,
		ADC_TypeDef* ADCx,
		DMA_TypeDef *DMAx,
		uint32_t dma_stream,
		uint32_t dma_channel,
		int32_t refMiliVolts,
		uint32_t resolution,
		uint32_t samplingTime,
		uint32_t injectedTrigger,
		uint32_t injectedTriggerEdge,
		RegConvMode regConvMode,
		uint32_t regularTrigger,
		uint32_t regularTriggerEdge,
		uint32_t irq_priority,
		uint32_t dma_irq_priority)
	: ADCx_(ADCx)
	, dma_(DMAx, dma_stream, dma_channel, LL_DMA_MODE_CIRCULAR | LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD | LL_DMA_PRIORITY_HIGH, dma_irq_priority)
	, refMiliVolts_(refMiliVolts)
	, resolution_(resolution)
	, injdataSize_(injChannels.size())
	, regdataSize_(regChannels.size())
{
	if (ADCx_ == ADC1) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	}
#ifdef ADC2
	else if (ADCx_ == ADC2) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);
	}
#endif
#ifdef ADC3
	else if (ADCx_ == ADC3) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC3);
	}
#endif

	/*
	 * Configure DMA transfer for regular channels
	 */
	dma_.ConfigAddresses(LL_ADC_DMA_GetRegAddr(ADCx_, LL_ADC_DMA_REG_REGULAR_DATA), (uint32_t)&regdata_raw_[0], LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	dma_.SetDataLength(regChannels.size());
	dma_.Callback_TC(this, &Adc::Callback_DmaTC);
	dma_.EnableIT_TC();

	for (auto& pin : pins)
		pin.Init();

	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADCx_), LL_ADC_CLOCK_SYNC_PCLK_DIV2);

	if (instance_count_) {
		auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(ADC_IRQn);
		InterruptManager::instance().Callback_EnableIRQ(ADC_IRQn, irq_priority, [=](void){IRQHandler(); OldIrqHandler();});
	} else {
		InterruptManager::instance().Callback_EnableIRQ(ADC_IRQn, irq_priority, [=](void){IRQHandler(); });
	}
	instance_count_++;

	LL_ADC_SetResolution(ADCx_, resolution);

	/* Set ADC conversion data alignment */
	LL_ADC_SetResolution(ADCx_, LL_ADC_DATA_ALIGN_RIGHT);

	InitInjectedChannels(samplingTime, injChannels);
	SetInjectedTrigger(injectedTrigger, injectedTriggerEdge);

	InitRegularChannels(samplingTime, regChannels);
	SetRegularTrigger(regularTrigger, regularTriggerEdge, regConvMode);
}

Adc::~Adc()
{
	Disable();
}

void Adc::SetInjectedTrigger(uint32_t injectedTrigger, uint32_t injectedTriggerEdge)
{
	if (injectedTrigger == LL_ADC_INJ_TRIG_SOFTWARE) {
	    LL_ADC_INJ_SetTriggerSource(ADCx_, LL_ADC_INJ_TRIG_SOFTWARE);
	} else {
	    LL_ADC_INJ_SetTriggerSource(ADCx_, injectedTrigger);
	    LL_ADC_INJ_StartConversionExtTrig(ADCx_, injectedTriggerEdge);
	    LL_ADC_INJ_SetTrigAuto(ADCx_, LL_ADC_INJ_TRIG_INDEPENDENT);
	}
}

void Adc::InitInjectedChannels(uint32_t samplingTime, const std::vector<uint32_t>& injChannels)
{
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

void Adc::SetRegularTrigger(uint32_t regularTrigger, uint32_t regularTriggerEdge, RegConvMode regConvMode)
{
	if (regularTrigger == LL_ADC_REG_TRIG_SOFTWARE) {
	    LL_ADC_REG_SetTriggerSource(ADCx_, LL_ADC_REG_TRIG_SOFTWARE);
	} else {
	    LL_ADC_REG_SetTriggerSource(ADCx_, regularTrigger);
	    LL_ADC_REG_StartConversionExtTrig(ADCx_, regularTriggerEdge);
	}

	/* Set ADC group regular continuous mode */
	SetRegConversionMode(regConvMode);
}

void Adc::InitRegularChannels(uint32_t samplingTime, const std::vector<uint32_t>& regChannels)
{
	/* Set ADC group regular conversion data transfer */
	LL_ADC_REG_SetDMATransfer(ADCx_, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

	/* Specify which ADC flag between EOC (end of unitary conversion)         */
	/* or EOS (end of sequence conversions) is used to indicate               */
	/* the end of conversion.                                                 */
	LL_ADC_REG_SetFlagEndOfConversion(ADCx_, LL_ADC_REG_FLAG_EOC_SEQUENCE_CONV);

	/* Set ADC group regular sequencer */
	/* Note: On this STM32 series, ADC group regular sequencer is              */
	/*       fully configurable: sequencer length and each rank               */
	/*       affectation to a channel are configurable.                       */
	/*       Refer to description of function                                 */
	/*       "LL_ADC_REG_SetSequencerLength()".                               */

	/* Set ADC channels sampling time */
	for (auto ch : regChannels) {
		LL_ADC_SetChannelSamplingTime(ADCx_, ch, samplingTime);
	}

	switch (regChannels.size()) {
	default:
	case 1:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_DISABLE);
		break;
	case 2:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);
		break;
	case 3:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS);
		break;
	case 4:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS);
		break;
	case 5:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS);
		break;
	case 6:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS);
		break;
	case 7:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_7RANKS);
		break;
	case 8:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_8RANKS);
		break;
	case 9:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_9RANKS);
		break;
	case 10:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_10RANKS);
		break;
	case 11:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_11RANKS);
		break;
	case 12:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_12RANKS);
		break;
	case 13:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_13RANKS);
		break;
	case 14:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_14RANKS);
		break;
	case 15:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_15RANKS);
		break;
	case 16:
		LL_ADC_REG_SetSequencerLength(ADCx_, LL_ADC_REG_SEQ_SCAN_ENABLE_16RANKS);
		break;
	};

	/* Set ADC group injected sequencer length and scan direction */
	if (regChannels.size() >= 1) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_1, regChannels[0]);
	}
	if (regChannels.size() >= 2) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_2, regChannels[1]);
	}
	if (regChannels.size() >= 3) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_3, regChannels[2]);
	}
	if (regChannels.size() >= 4) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_4, regChannels[3]);
	}
	if (regChannels.size() >= 5) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_5, regChannels[4]);
	}
	if (regChannels.size() >= 6) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_6, regChannels[5]);
	}
	if (regChannels.size() >= 7) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_7, regChannels[6]);
	}
	if (regChannels.size() >= 8) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_8, regChannels[7]);
	}
	if (regChannels.size() >= 9) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_9, regChannels[8]);
	}
	if (regChannels.size() >= 10) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_10, regChannels[9]);
	}
	if (regChannels.size() >= 11) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_11, regChannels[10]);
	}
	if (regChannels.size() >= 12) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_12, regChannels[11]);
	}
	if (regChannels.size() >= 13) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_13, regChannels[12]);
	}
	if (regChannels.size() >= 14) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_14, regChannels[13]);
	}
	if (regChannels.size() >= 15) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_15, regChannels[14]);
	}
	if (regChannels.size() >= 16) {
		LL_ADC_REG_SetSequencerRanks(ADCx_, LL_ADC_REG_RANK_16, regChannels[15]);
	}
}

int32_t Adc::GetRegularData(size_t idx)
{
	return __LL_ADC_CALC_DATA_TO_VOLTAGE(refMiliVolts_, regdata_raw_[idx], resolution_);
}

int32_t Adc::SWTrigGetRegularData(size_t idx)
{
	RegularSWTrigger();
	while (IsRegConversionInProgress())
		;
	return GetRegularData(idx);
}

void Adc::RegularSWTrigger()
{
	dma_inproggress_ = true;
	LL_ADC_REG_StartConversionSWStart(ADCx_);
}

void Adc::InjectedSWTrigger()
{
	LL_ADC_INJ_StartConversionSWStart(ADCx_);
}


void Adc::Enable()
{
	dma_.Enable();
	LL_ADC_Enable(ADCx_);
}

void Adc::Disable()
{
	dma_.Disable();
	LL_ADC_Disable(ADCx_);
}

void Adc::Callback_DmaTC(void)
{
	dma_inproggress_ = false;
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
	injdata_[0] = __LL_ADC_CALC_DATA_TO_VOLTAGE(refMiliVolts_, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_1), resolution_);
	injdata_[1] = __LL_ADC_CALC_DATA_TO_VOLTAGE(refMiliVolts_, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_2), resolution_);
	injdata_[2] = __LL_ADC_CALC_DATA_TO_VOLTAGE(refMiliVolts_, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_3), resolution_);
	injdata_[3] = __LL_ADC_CALC_DATA_TO_VOLTAGE(refMiliVolts_, LL_ADC_INJ_ReadConversionData12(ADCx_, LL_ADC_INJ_RANK_4), resolution_);
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

