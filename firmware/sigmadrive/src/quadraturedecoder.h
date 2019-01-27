#ifndef QUADRATUREDECODER_H_
#define QUADRATUREDECODER_H_

#include <stdint.h>
#include <vector>
#include <functional>
#include "timer.h"
#include "digitalin.h"
#include "gpiopin.h"


class QuadratureDecoder: public Timer {
public:
	using base = Timer;

	QuadratureDecoder(
			TIM_TypeDef *TIMx,
			uint32_t counter_max,
			uint32_t irq_priority = 0,
			const std::vector<GPIOPin>& pins = {}
	);

	virtual ~QuadratureDecoder();

	void Start();

	uint32_t GetPosition();
	void ResetPosition(uint32_t position);

protected:
	uint32_t counter_max_;
};

#endif
