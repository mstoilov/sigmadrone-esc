#include "quadraturedecoder.h"

QuadratureDecoder::QuadratureDecoder(TIM_TypeDef *TIMx,	uint32_t counter_max, uint32_t irq_priority, const std::vector<GPIOPin>& pins)
	: Timer(TIMx, TimeSpan(0), Frequency::from_hertz(SystemCoreClock), irq_priority, pins)
{
	SetAutoReloadValue(counter_max);
	SetPrescaler(0);
	SetClockDivision(0);
	SetCounterMode(Up);
	SetRepetionCounterValue(0);

	SetEncoderMode(EncoderModeTI12);

	SetICActiveInput(CH1, DirectTI);
	SetICFilter(CH1, FilterDiv1);
	SetICPrescaler(CH1, PrescalerDiv1);
	SetICPolarity(CH1, PolarityRising);

	SetICActiveInput(CH2, IndirectTI);
	SetICFilter(CH2, FilterDiv1);
	SetICPrescaler(CH2, PrescalerDiv1);
	SetICPolarity(CH2, PolarityFalling);

}

QuadratureDecoder::~QuadratureDecoder()
{
}

void QuadratureDecoder::Start()
{
	EnableChannel(CH1);
	EnableChannel(CH2);
	base::Start();
}
