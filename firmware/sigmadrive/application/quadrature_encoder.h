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
	QuadratureEncoder(uint32_t cpr_max);
	virtual ~QuadratureEncoder();
	void Attach(TIM_HandleTypeDef* htim);
	uint32_t GetMaxCounter();
	void InvalidateIndexOffset();
	void SetIndexOffset(int32_t cpr);
	void CallbackIndex();

public:
	virtual void Start() override;
	virtual void Stop() override;
	virtual uint32_t GetCounter() override;
	virtual uint32_t GetMaxPosition() override;
	virtual void ResetPosition() override;
	virtual uint32_t GetPosition() override;
	virtual uint32_t GetRevolutions() override;
	virtual uint32_t GetIndexPosition() override;
	virtual float GetElectricPosition(uint32_t position, uint32_t motor_pole_pairs) override;
	virtual float GetMechanicalPosition(uint32_t position) override;
	virtual uint32_t GetLastError() override { return 0; }
	virtual bool Update() override { return true; }
	virtual bool UpdateBegin() override { return true; }
	virtual bool UpdateEnd() override { return true; }

private:
	void SetCounter(uint32_t cpr);

protected:
	uint32_t cpr_max_;
	volatile int32_t index_offset_;
	TIM_HandleTypeDef* htim_;
};

#endif /* _QUADRATUREENCODER_H_ */
