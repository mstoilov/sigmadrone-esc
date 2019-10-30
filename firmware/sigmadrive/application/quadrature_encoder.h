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

#include "iencoder.h"

class QuadratureEncoder : public IEncoder {
public:
	QuadratureEncoder(uint32_t cpr_max, uint32_t motor_pole_pairs);
	virtual ~QuadratureEncoder();
	void Attach(TIM_HandleTypeDef* htim);
	uint32_t GetMaxCounter();
	void InvalidateIndexOffset();
	void SetIndexOffset(int32_t cpr);
	int32_t GetIndexOffset();
	void CallbackIndex();

public:
	virtual void Start() override;
	virtual void Stop() override;
	virtual uint32_t GetCountsPerRotation() override;
	virtual void ResetCounter(uint32_t cpr) override;
	virtual uint32_t GetCounter();
	virtual uint32_t GetMotorPolePairs() override;
	virtual void SetMotorPolePairs(uint32_t motor_pole_pairs) override;
	virtual float GetElectricPosition() override;
	virtual float GetMechanicalPosition() override;
	virtual RpcProperty GetProperties() override;

protected:
	uint32_t cpr_max_;
	uint32_t motor_pole_pairs_;
	int32_t index_offset_;
	TIM_HandleTypeDef* htim_;
};

#endif /* _QUADRATUREENCODER_H_ */
