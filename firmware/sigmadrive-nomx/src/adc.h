#ifndef ADC_H_
#define ADC_H_

#include <vector>
#include <functional>
#include "sigmadrive.h"
#include "dma.h"

#include "gpiopin.h"

class Adc {
public:
	enum RegConvMode {
		RegConvModeSingle = LL_ADC_REG_CONV_SINGLE,
		RegConvModeContinuous = LL_ADC_REG_CONV_CONTINUOUS,
	};

	Adc(const std::vector<GPIOPin>& pins,
		const std::vector<uint32_t>& regChannels,
		const std::vector<uint32_t>& injChannels,
		ADC_TypeDef* ADCx = ADC1,
		DMA_TypeDef *DMAx = DMA2,
		uint32_t dma_stream = LL_DMA_STREAM_0,
		uint32_t dma_channel = LL_DMA_CHANNEL_0,
		int32_t refMiliVolts = 3300,
		uint32_t resolution = LL_ADC_RESOLUTION_12B,
		uint32_t samplingTime = LL_ADC_SAMPLINGTIME_3CYCLES,
		uint32_t injectedTrigger = LL_ADC_INJ_TRIG_SOFTWARE,
		uint32_t injectedTriggerEdge = LL_ADC_INJ_TRIG_EXT_RISING,
		RegConvMode regConvMode = RegConvModeSingle,
		uint32_t regularTrigger = LL_ADC_REG_TRIG_SOFTWARE,
		uint32_t regularTriggerEdge = LL_ADC_REG_TRIG_EXT_RISING,
		uint32_t irq_priority = 0,
		uint32_t dma_irq_priority = 0);
	virtual ~Adc();
	void Enable();
	void Disable();
	void SetInjectedTrigger(uint32_t trigger, uint32_t triggerEdge);
	void SetRegularTrigger(uint32_t trigger, uint32_t triggerEdge, RegConvMode regConvMode);
	void RegularSWTrigger();
	void InjectedSWTrigger();
	void SetRegConversionMode(RegConvMode mode)		{ LL_ADC_REG_SetContinuousMode(ADCx_, mode); }
	bool IsRegConversionInProgress()				{ return dma_inproggress_; }
	int32_t GetRegularDataRaw(size_t idx)			{ return regdata_raw_[idx]; }

	/*
	 * Return the ADC data in mV
	 */
	int32_t GetRegularData(size_t idx);

	/*
	 * SW Trigger and return the ADC data in mV
	 * This function will start the conversion and
	 * wait until the data is DMA in the buffer.
	 */
	int32_t SWTrigGetRegularData(size_t idx);


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
	int32_t refMiliVolts_;
	uint32_t resolution_;
	size_t injdataSize_;
	size_t regdataSize_;
	__IO bool dma_inproggress_ = false;
	__IO int16_t regdata_raw_[20];
	int32_t injdata_[4];
};

#endif
