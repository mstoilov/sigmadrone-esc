/*
 * QuadratureEncoder.cpp
 *
 *  Created on: Sep 6, 2019
 *      Author: mstoilov
 */

#include "quadrature_encoder.h"

QuadratureEncoder::QuadratureEncoder(uint32_t cpr_max)
	: cpr_max_(cpr_max)
	, index_offset_(-1)
{

}

QuadratureEncoder::~QuadratureEncoder()
{

}

void QuadratureEncoder::Attach(TIM_HandleTypeDef* htim)
{
	htim_ = htim;
	__HAL_TIM_SET_AUTORELOAD(htim_, cpr_max_);
}

void QuadratureEncoder::Start()
{
	HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
}

void QuadratureEncoder::Stop()
{
	HAL_TIM_Encoder_Stop(htim_, TIM_CHANNEL_ALL);
}

/*
 * Value will be wrapped within 0 to max CPR
 */
void QuadratureEncoder::ResetCounter(uint32_t cpr)
{
	__HAL_TIM_SET_COUNTER(htim_, cpr % cpr_max_);
}

uint32_t QuadratureEncoder::GetPosition()
{
	return (__HAL_TIM_GET_COUNTER(htim_) >> 2);
}

uint32_t QuadratureEncoder::GetMaxPosition()
{
	return cpr_max_ >> 2;
}

void QuadratureEncoder::CallbackIndex()
{
	if (index_offset_ < 0) {
		SetIndexOffset(__HAL_TIM_GET_COUNTER(htim_));
	}
	ResetCounter(index_offset_);
}

void QuadratureEncoder::InvalidateIndexOffset()
{
	SetIndexOffset(-1);
}

void QuadratureEncoder::SetIndexOffset(int32_t cpr)
{
	index_offset_ = cpr;
}

int32_t QuadratureEncoder::GetIndexOffset()
{
	return index_offset_;
}

