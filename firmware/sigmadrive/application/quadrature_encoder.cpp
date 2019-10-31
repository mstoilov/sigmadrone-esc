/*
 * QuadratureEncoder.cpp
 *
 *  Created on: Sep 6, 2019
 *      Author: mstoilov
 */

#include <math.h>
#include "quadrature_encoder.h"


QuadratureEncoder::QuadratureEncoder(uint32_t cpr_max, uint32_t motor_pole_pairs)
	: cpr_max_(cpr_max)
	, motor_pole_pairs_(motor_pole_pairs)
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

uint32_t QuadratureEncoder::GetCountsPerRotation()
{
	return GetMaxCounter();
}

/*
 * Value will be wrapped within 0 to max CPR
 */
void QuadratureEncoder::ResetCounter(uint32_t cpr)
{
	__HAL_TIM_SET_COUNTER(htim_, cpr % cpr_max_);
}

uint32_t QuadratureEncoder::GetCounter()
{
	return (__HAL_TIM_GET_COUNTER(htim_) >> 2);
}

uint32_t QuadratureEncoder::GetMaxCounter()
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

uint32_t QuadratureEncoder::GetMotorPolePairs()
{
	return motor_pole_pairs_;
}

void QuadratureEncoder::SetMotorPolePairs(uint32_t motor_pole_pairs)
{
	motor_pole_pairs_ = motor_pole_pairs;
}

float QuadratureEncoder::GetElectricPosition()
{
	uint32_t cpr = GetCountsPerRotation();
	return 2.0f * M_PI * (GetCounter() % (cpr / motor_pole_pairs_)) / (cpr / motor_pole_pairs_);
}

float QuadratureEncoder::GetMechanicalPosition()
{
	uint32_t cpr = GetCountsPerRotation();
	return 2.0f * M_PI * (GetCounter() % (cpr)) / (cpr);
}

rexjson::property QuadratureEncoder::GetProperties()
{
	rexjson::property props = rexjson::property_map {
		{"motor_pole_pairs", rexjson::property(&motor_pole_pairs_)},
		{"cpr_max", rexjson::property(&cpr_max_)},
	};

	return props;
}
