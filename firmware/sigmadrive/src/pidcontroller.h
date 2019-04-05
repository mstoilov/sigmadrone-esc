#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <algorithm>

template <class T>
class PidController
{
public:
	PidController(float kp = 0, float ki = 0, float kd = 0, float integral_leak = 0)
		: integral_error_(0)
		, last_error_input_(0)
		, kp_(kp)
		, ki_(ki)
		, kd_(kd)
		, integral_leak_(0)
	{
		SetIntegralLeak(integral_leak);
	}

	void SetKp(float kp) 							{ kp_ = kp; }
	void SetKi(float ki) 							{ ki_ = ki; }
	void SetKd(float kd) 							{ kd_ = kd; }
	void SetIntegralLeak(float integral_leak)		{ integral_leak_ = 1.0f - std::min(std::max(integral_leak, 0.0f), 1.0f); }
	void SetKpid(float kp, float ki, float kd)		{ SetKp(kp); SetKi(ki); SetKd(kd); }
	float GetKp() const 							{ return kp_; }
	float GetKi() const 							{ return ki_; }
	float GetKd() const 							{ return kd_; }
	T GetOutput() const								{ return output_; }

	void Update(const T& error_input, float dT)
	{
		integral_error_ = (integral_error_ + error_input * dT) * integral_leak_;
		integral_error_ = (error_input - last_error_input_) / dT;
		last_error_input_ = error_input;
		output_ = kp_ * error_input + ki_ * integral_error_ + kd_ * differential_error_;
	}



private:
	T output_;				/* PID controller output. */
	T differential_error_;	/* First derivative of the error input. */
	T integral_error_;		/* accumulated integral error */
	T last_error_input_;	/* error input from the previous iteration */
	float kp_;				/* proportional gain */
	float ki_;				/* integral gain */
	float kd_;				/* derivative gain */
	float integral_leak_;	/* Integral error leak. This must be between 0.0 and 1.0 */

};

#endif /* PIDCONTROLLER_H_ */
