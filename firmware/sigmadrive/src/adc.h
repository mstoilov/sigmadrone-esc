#ifndef ADC_H_
#define ADC_H_

#include <vector>
#include <functional>
#include "stm32f4xx_ll_adc.h"

#include "gpiopin.h"

class Adc {
public:
	Adc(ADC_TypeDef* ADCx = ADC1,
			uint32_t resolution = LL_ADC_RESOLUTION_12B,
			uint32_t samplingTime = LL_ADC_SAMPLINGTIME_3CYCLES,
			uint32_t injectedTrigger = LL_ADC_INJ_TRIG_SOFTWARE,
			const std::vector<GPIOPin>& pins = {},
			const std::vector<uint32_t>& injChannels = {});
	virtual ~Adc();
	void Start();
	void Stop();
	void SetInjectedTrigger(uint32_t trigger);

	template<typename T>
	void Callback_PWMCC(T* tptr, void (T::*mptr)(uint32_t*, uint32_t))
	{
		callback_JEOS_ = [=](uint32_t* data, uint32_t size){(tptr->*mptr)(data, size);};
	}

	void CallbackJEOS(const std::function<void(int32_t*, size_t)>& callback)
	{
		callback_JEOS_ = callback;
	}

protected:
	void InitInjectedChannels(uint32_t resolution, uint32_t samplingTime, const std::vector<uint32_t>& injChannels);
	void IRQHandler(void);
	virtual void handle_overrun();
	virtual void handle_eocs();
	virtual void handle_jeos();
	virtual void handle_awd1();

	std::function<void(int32_t*, size_t)> callback_JEOS_ = [](int32_t*, size_t){};

public:
	ADC_TypeDef* ADCx_;
	uint32_t resolution_;
	size_t injdataSize_;
	int32_t injdata_[4];
};

#endif
