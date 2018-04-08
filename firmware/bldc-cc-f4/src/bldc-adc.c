#include "bldc-adc.h"
#include "bldc-config.h"


void bldc_adc_init(ADC_TypeDef *ADCx)
{
	if (LL_ADC_IsEnabled(ADCx) == 0)
	{
		/* Set ADC clock (conversion clock) common to several ADC instances */
		LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADCx), LL_ADC_CLOCK_SYNC_PCLK_DIV2);
	}

	if (LL_ADC_IsEnabled(ADCx) == 0)
	{
		/* Set ADC data resolution */
		LL_ADC_SetResolution(ADCx, LL_ADC_RESOLUTION_12B);

		/* Set ADC conversion data alignment */
		LL_ADC_SetResolution(ADCx, LL_ADC_DATA_ALIGN_RIGHT);

		/* Set Set ADC sequencers scan mode, for all ADC groups                   */
		/* (group regular, group injected).                                       */
//		LL_ADC_SetSequencersScanMode(ADCx, LL_ADC_SEQ_SCAN_DISABLE);

	    /* Set Set ADC sequencers scan mode, for all ADC groups                   */
	    /* (group regular, group injected).                                       */
	    LL_ADC_SetSequencersScanMode(ADCx, LL_ADC_SEQ_SCAN_ENABLE);

	    /* Set ADC channels sampling time */
	    LL_ADC_SetChannelSamplingTime(ADCx, CHANNEL_PHASE_A, ADC_SAMPLING_CYCLES);
	    LL_ADC_SetChannelSamplingTime(ADCx, CHANNEL_PHASE_B, ADC_SAMPLING_CYCLES);
	    LL_ADC_SetChannelSamplingTime(ADCx, CHANNEL_PHASE_C, ADC_SAMPLING_CYCLES);
	    LL_ADC_SetChannelSamplingTime(ADCx, CHANNEL_CENTER, ADC_SAMPLING_CYCLES);

//	    LL_ADC_INJ_SetTriggerSource(ADCx, LL_ADC_INJ_TRIG_EXT_TIM1_CH4);
	    LL_ADC_INJ_SetTriggerSource(ADCx, LL_ADC_INJ_TRIG_EXT_TIM2_CH1);
	    LL_ADC_INJ_StartConversionExtTrig(ADCx, LL_ADC_INJ_TRIG_EXT_FALLING);
	    LL_ADC_INJ_SetTrigAuto(ADCx, LL_ADC_INJ_TRIG_INDEPENDENT);

		/* Set ADC group injected sequencer length and scan direction */
		LL_ADC_INJ_SetSequencerLength(ADCx, LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS);

		/* Set ADC group injected sequencer discontinuous mode */
		LL_ADC_INJ_SetSequencerDiscont(ADCx, LL_ADC_INJ_SEQ_DISCONT_DISABLE);

		if (LL_ADC_INJ_GetSequencerLength(ADCx) == LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS) {
		/* Set ADC group injected sequence: channel on the selected sequence rank. */
			LL_ADC_INJ_SetSequencerRanks(ADCx, LL_ADC_INJ_RANK_1, CHANNEL_PHASE_A);
			LL_ADC_INJ_SetSequencerRanks(ADCx, LL_ADC_INJ_RANK_2, CHANNEL_PHASE_B);
			LL_ADC_INJ_SetSequencerRanks(ADCx, LL_ADC_INJ_RANK_3, CHANNEL_PHASE_C);
			LL_ADC_INJ_SetSequencerRanks(ADCx, LL_ADC_INJ_RANK_4, CHANNEL_CURRENT_ADC);
		} else if (LL_ADC_INJ_GetSequencerLength(ADCx) == LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS) {
			LL_ADC_INJ_SetSequencerRanks(ADCx, LL_ADC_INJ_RANK_2, CHANNEL_PHASE_A);
			LL_ADC_INJ_SetSequencerRanks(ADCx, LL_ADC_INJ_RANK_3, CHANNEL_PHASE_B);
			LL_ADC_INJ_SetSequencerRanks(ADCx, LL_ADC_INJ_RANK_4, CHANNEL_PHASE_C);
		}

		LL_ADC_EnableIT_JEOS(ADCx);
	}


}

void bldc_adc_start(ADC_TypeDef *ADCx)
{
	LL_ADC_Enable(ADCx);

}

void bldc_adc_stop(ADC_TypeDef *ADCx)
{
	LL_ADC_Disable(ADCx);

}
