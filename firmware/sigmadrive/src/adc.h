#ifndef ADC_H_
#define ADC_H_

#include <vector>
#include <functional>
#include "stm32f4xx_ll_adc.h"

#include "gpiopin.h"

class Adc {
public:
	Adc(ADC_TypeDef* ADCx, const std::vector<GPIOPin>& pins = {});
	virtual ~Adc();
	void Start();
	void Stop();

protected:
	void InitCurrentFeedback();
	void IRQHandler(void);
	virtual void handle_overrun();
	virtual void handle_eocs();
	virtual void handle_jeos();
	virtual void handle_awd1();


public:
	ADC_TypeDef* ADCx_;
	int32_t injdata_[4];
};

#endif
