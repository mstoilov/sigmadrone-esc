#ifndef _IPWMGENERATOR_H_
#define _IPWMGENERATOR_H_

#include <stdint.h>
#include <stddef.h>

class IPwmGenerator {
public:
	virtual ~IPwmGenerator() {};

	/** Enable the timer channels.
	 *
	 * This enables the outputs of the timer. By calling
	 * this function the outputs of the timer channels
	 * will be applied to the power FETS gates.
	 */
	virtual void Start() = 0;

	/** Disable the timer channels.
	 *
	 * This disables the outputs of the timer. By calling
	 * this function the outputs of the timer channels
	 * will be removed from the power FETS gates.
	 */
	virtual void Stop() = 0;

	/** Get the current value of the counter
	 *
	 * @return counter value
	 */
	virtual uint32_t GetCounter() = 0;

	/** Get the counter direction
	 *
	 * @return Return 0 if the counter is going up,
	 * return 1 if the counter is going down
	 */
	virtual uint32_t GetCounterDirection() = 0;

	/** Return the status of the timer counter
	 *
	 * @return true if the counter is enabled, false if it is disabled
	 */
	virtual bool IsStarted() = 0;

	/** Get the PWM period in clock units.
	 *
	 */
	virtual uint32_t GetPeriod() = 0;

	/** Set the PWM period in clock units.
	 *
	 * @param period Timer period
	 */
	virtual void SetPeriod(uint32_t period) = 0;


	/** Get the PWM timings for the different channels in clock units.
	 *
	 * @param values Timing values buffer
	 * @param count buffer size
	 */
	virtual void GetTimings(uint32_t* values, size_t count) = 0;

	/** Set the PWM timings for the different channels in clock units.
	 *
	 * @param values Timing values buffer
	 * @param count buffer size
	 */
	virtual void SetTimings(const uint32_t* values, size_t count) = 0;

	/** Get compare value set for the specified output channel
	 *
	 * @param channel The specified channel
	 * @return the compare value for the channel
	 */
	virtual uint32_t GetTiming(uint32_t channel) = 0;

	/** Set compare value for specified output channel
	 *
	 * The way hardware timers in PWM mode work is by constantly
	 * compare the "compare value" for the channel with the current
	 * value of the timer counter. When the compare value is equal to
	 * the timer counter the channel output will be flipped and that
	 * is how it generate the PWM.
	 *
	 * @param channel specified channel
	 * @param value Compare value
	 */
	virtual void SetTiming(uint32_t channel, uint32_t value) = 0;


public:


};

#endif /* _IPWMGENERATOR_H_ */
