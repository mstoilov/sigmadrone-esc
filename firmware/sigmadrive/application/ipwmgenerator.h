#ifndef _IPWMGENERATOR_H_
#define _IPWMGENERATOR_H_

#include <stdint.h>
#include <stddef.h>

class IPwmGenerator {
public:
	virtual ~IPwmGenerator() {};

	virtual void Start() = 0;
	virtual void Stop() = 0;
	virtual bool IsStarted() = 0;

	/*
	 * Get/Set the PWM period in clock units.
	 */
	virtual uint32_t GetPeriod() = 0;
	virtual void SetPeriod(uint32_t period) = 0;

	/*
	 * Get/Set the PWM timings for the different channels in clock units.
	 */
	virtual void GetTimings(uint32_t* values, size_t count) = 0;
	virtual void SetTimings(const uint32_t* values, size_t count) = 0;
	virtual uint32_t GetTiming(uint32_t channel) = 0;
	virtual void SetTiming(uint32_t channel, uint32_t value) = 0;


public:


};

#endif /* _IPWMGENERATOR_H_ */
