/*
 * quadrature_encoder.h
 *
 *  Created on: Sep 6, 2019
 *      Author: mstoilov
 */

#ifndef _QUADRATUREENCODER_H_
#define _QUADRATUREENCODER_H_


#include <map>
#include <functional>
#include "stm32f745xx.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_tim.h"


class QuadratureEncoder {
public:
	QuadratureEncoder(uint32_t cpr_max);
	virtual ~QuadratureEncoder();
	void Attach(TIM_HandleTypeDef* htim);

	void Start();
	void Stop();
	void ResetCounter(uint32_t cpr);
	uint32_t GetPosition();
	void ResetPosition(uint32_t position);
	uint32_t GetMaxPosition();
	void InvalidateIndexOffset();
	void SetIndexOffset(int32_t cpr);
	int32_t GetIndexOffset();
	void CallbackIndex();


protected:
	uint32_t cpr_max_;
	int32_t index_offset_;
	TIM_HandleTypeDef* htim_;
};

#endif /* _QUADRATUREENCODER_H_ */
