#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

template <class T>
class PidController
{
public:
	PidController(float kp = 0, float ki = 0, float kd = 0)
		: integral_err_(0)
		, last_derivative_err_(0)
		, last_err_input_(0)
		, kp_(kp)
		, ki_(ki)
		, kd_(kd),
	{
	}

	void SetKp(float kp) 							{ kp_ = kp; }
	void SetKi(float ki) 							{ ki_ = ki; }
	void SetKd(float kd) 							{ kd_ = kd; }
	void SetKpKiKd(float kp, float ki, float kd)	{ SetKp(kp); SetKi(ki); SetKd(kd); }
	float GetKp() const 							{ return kp_; }
	float GetKi() const 							{ return ki_; }
	float GetKd() const 							{ return kd_; }

private:
	T integral_err_;        /* accumulated integral error */
	T last_derivative_err_; /* derivative err from the previous iteration */
	T last_err_input_;      /* error input from the previous iteration */
	float kp_;              /* proportional gain */
	float ki_;              /* integral gain */
	float kd_;              /* derivative gain */

};

#endif /* PIDCONTROLLER_H_ */
