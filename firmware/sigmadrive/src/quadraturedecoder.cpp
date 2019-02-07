#include "quadraturedecoder.h"

QuadratureDecoder::QuadratureDecoder(TIM_TypeDef *TIMx,	uint32_t max_cpr, uint32_t irq_priority, const std::vector<GPIOPin>& pins)
	: Timer(TIMx, TimeSpan(0), Frequency::from_hertz(SystemCoreClock), irq_priority, pins)
	, counter_max_(max_cpr)
{
	SetAutoReloadValue(max_cpr);
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

/*
 * Position is: counter / 4
 */
uint32_t QuadratureDecoder::GetPosition()
{
	return (GetCounterValue() >> 2);
}

/*
 * Value will be wrapped within 0 to max CPR
 */
void QuadratureDecoder::ResetCounter(uint32_t value)
{
	SetCounterValue(value % counter_max_);
}

/*
 * Counter is: postion * 4
 */
void QuadratureDecoder::ResetPosition(uint32_t position)
{
	ResetCounter(position << 2);
}

uint32_t QuadratureDecoder::GetMaxPosition()
{
	return counter_max_ >> 2;
}

void QuadratureDecoder::CallbackIndex()
{
	if (index_offset_ < 0) {
		SetIndexOffset(GetCounterValue());
	}
	ResetCounter(index_offset_);
}

void QuadratureDecoder::SetIndexOffset(int32_t offset)
{
	index_offset_ = offset;
}

int32_t QuadratureDecoder::GetIndexOffset()
{
	return index_offset_;
}
