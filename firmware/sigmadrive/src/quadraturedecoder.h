#ifndef QUADRATUREDECODER_H_
#define QUADRATUREDECODER_H_

#include <stdint.h>
#include <vector>
#include <functional>
#include "timer.h"
#include "gpiopin.h"


class QuadratureDecoder: public Timer {
public:
	using base = Timer;

	QuadratureDecoder(
			TIM_TypeDef *TIMx,
			uint32_t counter_max,
			const std::vector<GPIOPin>& pins = {}
	);
	virtual ~QuadratureDecoder();

	void Start();
};

#endif
