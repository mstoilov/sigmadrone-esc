/*
 * pidcontroller.h
 *
 *  Created on: Nov 8, 2019
 *      Author: mstoilov
 */

#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

template<typename T>
class PidController {
public:
	PidController(float kp = 0, float ki = 0, float kd = 0, float leak = 0)
	: kp_(kp)
	, ki_(ki)
	, kd_(kd)
	, leak_(leak)
	, output_i_max_(0)
	, last_input_(0)
	, output_p_(0)
	, output_i_(0)
	, output_d_(0)
	{

	}

	~PidController()
	{

	}

	T Input(const T& input, float dt)
	{
		output_p_ = input * kp_;

		output_i_ *= (1.0f - leak_);
		output_i_ += input * dt * ki_;
		if (output_i_max_ && output_i_ > output_i_max_)
			output_i_ = output_i_max_;
		if (output_i_max_ && output_i_ < -output_i_max_)
			output_i_ = -output_i_max_;
		output_d_ = (input - last_input_) * kd_ / dt;
		last_input_ = input;

		return Output();
	}

	T Output() const
	{
		return output_p_ + output_i_ + output_d_;
	}

	void SetMaxIntegralOutput(const T& output_max)
	{
		output_i_max_ = output_max;
	}

	T GetMaxIntegralOutput() const
	{
		return output_i_max_;
	}

	void SetLeakRate(float leak)
	{
		leak_ = leak;
	}

	float GetLeakRate() const
	{
		return leak_;
	}

	void SetGainP(float kp)
	{
		kp_ = kp;
	}

	void SetGainI(float ki)
	{
		ki_ = ki;
	}

	void SetGainD(float kd)
	{
		kd_ = kd;
	}

public:
	float kp_;
	float ki_;
	float kd_;
	float leak_;
	T output_i_max_;
	T last_input_;
	T output_p_;
	T output_i_;
	T output_d_;
};

#endif /* _PIDCONTROLLER_H_ */
