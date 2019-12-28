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
	PidController(float kp = 0, float ki = 0, float kd = 0, float decay = 0, const T& output_i_max = 0, const T& bias = 0)
	: kp_(kp)
	, ki_(ki)
	, kd_(kd)
	, decay_(decay)
	, output_i_max_(output_i_max)
	, bias_(bias)
	, last_input_(0)
	, output_p_(0)
	, output_i_(0)
	, output_d_(0)
	, output_b_(0)
	{

	}

	~PidController()
	{

	}

	T Input(const T& input, float dt)
	{
		float decay = (1.0f - decay_ * dt);

		output_b_ = bias_;
		output_p_ = input * kp_;
		if (decay > 0)
			output_i_ *= decay;
		output_i_ += input * ki_ * dt;
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
		return output_b_ + output_p_ + output_i_ + output_d_;
	}

	T OutputP() const
	{
		return output_p_;
	}

	T OutputI() const
	{
		return output_i_;
	}

	T OutputD() const
	{
		return output_d_;
	}

	T OutputB() const
	{
		return output_b_;
	}

	void SetMaxIntegralOutput(const T& output_max)
	{
		output_i_max_ = output_max;
	}

	T GetMaxIntegralOutput() const
	{
		return output_i_max_;
	}

	void SetBias(const T& bias)
	{
		bias_ = bias;
	}

	T GetBias() const
	{
		return bias_;
	}


	void SetDecayRate(float decay)
	{
		decay_ = decay;
	}

	float GetDecayRate() const
	{
		return decay_;
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

	void Reset()
	{
		last_input_ = 0;
		output_p_ = 0;
		output_i_ = 0;
		output_d_ = 0;
	}

public:
	float kp_;
	float ki_;
	float kd_;
	float decay_;
	T output_i_max_;
	T bias_;
	T last_input_;
	T output_p_;
	T output_i_;
	T output_d_;
	T output_b_;
};

#endif /* _PIDCONTROLLER_H_ */
