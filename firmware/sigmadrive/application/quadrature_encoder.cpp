/*
 * QuadratureEncoder.cpp
 *
 *  Created on: Sep 6, 2019
 *      Author: mstoilov
 */

#include <math.h>
#include "quadrature_encoder.h"


inline uint32_t CprToPosition(uint32_t cpr)
{
	return cpr >> 2;
}

inline uint32_t PositionToCpr(uint32_t position)
{
	return position << 2;
}


QuadratureEncoder::QuadratureEncoder(uint32_t cpr_max)
	: cpr_max_(cpr_max)
	, index_offset_(-1)
	, htim_(NULL)
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
void QuadratureEncoder::SetCounter(uint32_t cpr)
{
	__HAL_TIM_SET_COUNTER(htim_, cpr % cpr_max_);
}

/*
 * Return Counts up to max CPR
 */
uint32_t QuadratureEncoder::GetCounter()
{
	return __HAL_TIM_GET_COUNTER(htim_);
}

uint32_t QuadratureEncoder::GetMaxCounter()
{
	return cpr_max_;
}


void QuadratureEncoder::ResetPosition()
{
	SetCounter(0);
	InvalidateIndexOffset();
}

uint64_t QuadratureEncoder::GetPosition()
{
	return CprToPosition(GetCounter());
}

uint32_t QuadratureEncoder::GetRevolutions()
{
	return 0;
}

uint32_t QuadratureEncoder::GetMaxRotation()
{
	return CprToPosition(GetMaxCounter());
}

void QuadratureEncoder::CallbackIndex()
{
	if (index_offset_ < 0) {
		SetIndexOffset(GetCounter());
	}
	SetCounter(index_offset_);
}

void QuadratureEncoder::InvalidateIndexOffset()
{
	SetIndexOffset(-1);
}

void QuadratureEncoder::SetIndexOffset(int32_t cpr)
{
	index_offset_ = cpr;
}

uint32_t QuadratureEncoder::GetIndexPosition()
{
	if (index_offset_ < 0)
		return -1;
	return CprToPosition(index_offset_);
}

float QuadratureEncoder::GetElectricPosition(uint64_t position, uint32_t motor_pole_pairs)
{
	uint32_t max_position = GetMaxRotation();
	return 2.0f * M_PI * (position % (max_position / motor_pole_pairs)) / (max_position / motor_pole_pairs);
}


float QuadratureEncoder::GetMechanicalPosition(uint64_t position)
{
	uint32_t max_postion = GetMaxRotation();
	return 2.0f * M_PI * (position % (max_postion)) / (max_postion);
}

