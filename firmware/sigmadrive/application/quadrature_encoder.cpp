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


QuadratureEncoder::QuadratureEncoder(uint32_t cpr_max, uint32_t motor_pole_pairs)
	: cpr_max_(cpr_max)
	, motor_pole_pairs_(motor_pole_pairs)
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


void QuadratureEncoder::ResetPosition(uint32_t position)
{
	SetCounter(PositionToCpr(position));
	InvalidateIndexOffset();
}


uint32_t QuadratureEncoder::GetPosition()
{
	return CprToPosition(GetCounter());
}

uint32_t QuadratureEncoder::GetMaxPosition()
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

int32_t QuadratureEncoder::GetIndexPosition()
{
	return CprToPosition(index_offset_);
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
	uint32_t max_position = GetMaxPosition();
	return 2.0f * M_PI * (GetPosition() % (max_position / motor_pole_pairs_)) / (max_position / motor_pole_pairs_);
}

float QuadratureEncoder::GetMechanicalPosition()
{
	uint32_t max_postion = GetMaxPosition();
	return 2.0f * M_PI * (GetPosition() % (max_postion)) / (max_postion);
}

rexjson::property QuadratureEncoder::GetProperties()
{
	rexjson::property props = rexjson::property_map {
		{"motor_pole_pairs", rexjson::property(&motor_pole_pairs_)},
		{"cpr_max", rexjson::property(&cpr_max_)},
	};

	return props;
}
