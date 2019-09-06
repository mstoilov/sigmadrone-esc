#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <algorithm>

template <class T>
class PidController
{
public:
	PidController(float kp = 0, float ki = 0, float kd = 0, float kl = 0)
		: output_(0)
		, differential_error_(0)
		, integral_error_(0)
		, last_error_input_(0)
		, kp_(kp)
		, ki_(ki)
		, kd_(kd)
		, kl_(kl)
	{
		SetKl(kl);
	}

	void SetKp(float kp) 							{ kp_ = kp; }
	void SetKi(float ki) 							{ ki_ = ki; }
	void SetKd(float kd) 							{ kd_ = kd; }
	void SetKl(float kl)							{ kl_ = std::min(std::max(kl, 0.0f), 1.0f); }
	void SetKpid(float kp, float ki, float kd)		{ SetKp(kp); SetKi(ki); SetKd(kd); }
	float GetKp() const 							{ return kp_; }
	float GetKi() const 							{ return ki_; }
	float GetKd() const 							{ return kd_; }
	float GetKl() const								{ return kl_; }
	T Output() const								{ return output_; }
	T Input(const T& error_input, float dT)
	{
		integral_error_ = (integral_error_ + error_input * dT) * (1.0 - kl_);
		differential_error_ = (error_input - last_error_input_) / dT;
		last_error_input_ = error_input;
		output_ = kp_ * error_input + ki_ * integral_error_ + kd_ * differential_error_;
		return Output();
	}



private:
	T output_;				/* PID controller output. */
	T differential_error_;	/* First derivative of the error input. */
	T integral_error_;		/* accumulated integral error */
	T last_error_input_;	/* error input from the previous iteration */
	float kp_;				/* proportional gain */
	float ki_;				/* integral gain */
	float kd_;				/* derivative gain */
	float kl_;				/* Integral error leak. This must be between 0.0 and 1.0 */

};

#endif /* PIDCONTROLLER_H_ */
