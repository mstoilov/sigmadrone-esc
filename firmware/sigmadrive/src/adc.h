#ifndef ADC_H_
#define ADC_H_

#include <vector>
#include <functional>
#include "sigmadrive.h"
#include "dma.h"

#include "gpiopin.h"

class Adc {
public:
	Adc(const std::vector<GPIOPin>& pins,
		const std::vector<uint32_t>& regChannels,
		const std::vector<uint32_t>& injChannels,
		ADC_TypeDef* ADCx = ADC1,
		DMA_TypeDef *DMAx = DMA2,
		uint32_t dma_stream = LL_DMA_STREAM_0,
		uint32_t dma_channel = LL_DMA_CHANNEL_0,
		uint32_t resolution = LL_ADC_RESOLUTION_12B,
		uint32_t samplingTime = LL_ADC_SAMPLINGTIME_3CYCLES,
		uint32_t injectedTrigger = LL_ADC_INJ_TRIG_SOFTWARE,
		uint32_t injectedTriggerEdge = LL_ADC_INJ_TRIG_EXT_RISING,
		uint32_t regularTrigger = LL_ADC_REG_TRIG_SOFTWARE,
		uint32_t regularTriggerEdge = LL_ADC_REG_TRIG_EXT_RISING,
		uint32_t irq_priority = 0);
	virtual ~Adc();
	void Enable();
	void Disable();
	void SetInjectedTrigger(uint32_t trigger, uint32_t triggerEdge);
	void SetRegularTrigger(uint32_t trigger, uint32_t triggerEdge);
	void RegularSWTrigger();
	void InjectedSWTrigger();
	int32_t GetRegularData(size_t idx) { return regdata_[idx]; }
	int32_t GetRegularDataConv(size_t idx) { return regdata_conv_[idx]; }

	template<typename T>
	void CallbackJEOS(T* tptr, void (T::*mptr)(int32_t*, size_t))
	{
		callback_JEOS_ = [=](int32_t* data, size_t size){(tptr->*mptr)(data, size);};
	}

	void CallbackJEOS(const std::function<void(int32_t*, size_t)>& callback)
	{
		callback_JEOS_ = callback;
	}

protected:
	void InitInjectedChannels(uint32_t samplingTime, const std::vector<uint32_t>& injChannels);
	void InitRegularChannels(uint32_t samplingTime, const std::vector<uint32_t>& injChannels);

	void Callback_DmaTC(void);
	void IRQHandler(void);
	virtual void handle_overrun();
	virtual void handle_eocs();
	virtual void handle_jeos();
	virtual void handle_awd1();

	std::function<void(int32_t*, size_t)> callback_JEOS_ = [](int32_t*, size_t){};
	static unsigned int instance_count_;

public:
	ADC_TypeDef* ADCx_;
	Dma dma_;
	uint32_t resolution_;
	size_t injdataSize_;
	__IO int16_t regdata_[20];
	__IO int32_t regdata_conv_[20];
	int32_t injdata_[4];
};

#endif
