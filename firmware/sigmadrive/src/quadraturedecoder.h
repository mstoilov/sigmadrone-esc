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
			const std::vector<GPIOPin>& pins,
			TIM_TypeDef *TIMx,
			uint32_t counter_max,
			uint32_t irq_priority = 0
	);

	virtual ~QuadratureDecoder();

	void Start();

	void ResetCounter(uint32_t cpr);
	uint32_t GetPosition();
	void ResetPosition(uint32_t position);
	uint32_t GetMaxPosition();
	void SetIndexOffset(int32_t cpr);
	int32_t GetIndexOffset();
	void CallbackIndex();


protected:
	/* The maximum value the counter can have. */
	uint32_t cpr_max_;

	/*
	 * The offset of the index output from the encoder.
	 * If this value is valid, i.e. not negative, the counter
	 * will be reset to this value every time the CallbackIndex is called.
	 */
	int32_t index_offset_;

};

#endif
