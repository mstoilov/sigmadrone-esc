/*
 * pwm6step.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mstoilov
 */

#ifndef PWM6STEP_H_
#define PWM6STEP_H_

#include <stdint.h>
#include <vector>
#include <sstream>
#include "timer.h"
#include "digitalout.h"
#include "stm32f4xx_ll_adc.h"

#undef IHM08M1

class PWM6Step : public Timer {
public:
	typedef Timer base;

	static constexpr unsigned int SINE_STATES = 6;
	static constexpr unsigned int MAX_HERTZ = 150;
	static constexpr unsigned int MIN_MILLIHERTZ = 3000;
	static constexpr unsigned int MECHANICAL_DEGREES_RATIO = 8;
	static constexpr unsigned int BEMF_INTEGRAL_THRESHOLD = 3300;
	static constexpr unsigned int ADC_A = 0;
	static constexpr unsigned int ADC_B = 1;
	static constexpr unsigned int ADC_C = 2;
	static constexpr float MAX_THROTTLE = 0.8;
	static constexpr unsigned int adc_data_counter1 = 1;
	static constexpr unsigned int adc_data_counter2 = 5;
	static constexpr unsigned int adc_data_size = 3;
	static constexpr unsigned int MAX_BEMF_BUFFER = 20;

	struct BemfMeasurement {
		int32_t bemf = 0;
		int32_t counter = 0;
	};

	enum JeosState {
		JEOS_STATE_MEASUREMENT1 = 0,
		JEOS_STATE_MEASUREMENT2 = 1,
		JEOS_STATE_ZERODETECTED = 2,
	};

	struct LogEntry {
		uint32_t serial_ = 0;
		uint32_t state_ = 0;
		uint32_t counter_ = 0;
		uint32_t log_counter_ = 0;
		int32_t	bemf_mslope_ = 0;
		int32_t	bemf_intercept_ = 0;
		int32_t zero_crossing_ = 0;
		int32_t integral_bemf_ = 0;
		int32_t adc_data1_[adc_data_size];
		int32_t adc_data2_[adc_data_size];
	};

	PWM6Step(
			TIM_TypeDef *TIMx,
			const Frequency& switching_freq,
			const Frequency& system_clock = Frequency::from_hertz(SystemCoreClock),
			TriggerOutput trigger_output = TrigUpdate,
			OCMode output = PWM1,
			OCPolarity polarity = High,
			OCPolarity npolarity = Low,
			uint32_t deadtime = 30, 		/* ns */
			uint32_t irq_priority = 0,
			const std::vector<GPIOPin>& output_pins = {}
	);
	virtual ~PWM6Step();
	virtual void Start();
	virtual void Stop();

	void SetupChannels(uint32_t state);
	float GetDutyCycle();
	void SetDutyCycle(float percent);
	void SetDutyPeriod(const TimeSpan& period);
	Frequency GetSwitchingFrequency();
	void Toggle();
	void CallbackJEOS(int32_t *injdata, size_t size);
	inline int32_t CalculateBEMF(int32_t v1, int32_t v2, int32_t v3);
	int32_t DetectBEMF(uint32_t state, int32_t *injdata);
	void GenerateComEvent();
	void LogComEvent();
	void SetJeosState(JeosState state);

	template<typename T>
	void CallbackCOM(T* tptr, void (T::*mptr)(void))
	{
		callback_COM_ = [=](void){(tptr->*mptr)();};
	}

	void CallbackCOM(const std::function<void(void)>& callback)
	{
		callback_COM_ = callback;
	}


public:
	OCMode pwm_mode_;
	OCPolarity polarity_;
	OCPolarity npolarity_;
	uint32_t com_cycles_max_;
	TimeSpan duty_;
	uint32_t state_ = 0;
	JeosState jeos_state_ = JEOS_STATE_MEASUREMENT1;
	uint32_t counter_ = 0;
	uint32_t log_counter_ = 0;
	int32_t	bemf_mslope_ = 0;
	int32_t	bemf_intercept_ = 0;
	int32_t zero_crossing_ = 0;
	int32_t integral_bemf_ = 0;
	BemfMeasurement msr_[2];
	int32_t adc_data1_[adc_data_size];
	int32_t adc_data2_[adc_data_size];
	LogEntry log_entry_;
	std::function<void(void)> callback_COM_ = [](){};

};


#endif /* PWM6STEP_H_ */
