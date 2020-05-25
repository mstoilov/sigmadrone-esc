
#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

/** PID controller implementation
 *
 * This is a straight forward implementation of PID controller
 * functionality. The implementation also supports max integral
 * output to prevent *integral windup*. Error decay rate and
 * bias output are also supported.
 *
 */
template<typename T>
class PidController {
public:
    /** Constructor
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Differential gain
     * @param output_i_max Maximum integral output limit
     * @param bias Output bias
     */
    PidController(float kp = 0, float ki = 0, float kd = 0, float alpha_d = 1.0, float output_max = 0, const T &bias = 0)
        : kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , alpha_d_(alpha_d)
        , output_max_((output_max < 0) ? -output_max : output_max)
        , bias_(bias)
        , last_error_(0)
        , output_p_(0)
        , output_i_(0)
        , output_d_(0)
        , output_b_(0)
    {
    }

    ~PidController()
    {
    }

    /** Input method
     *
     * This method is used to recalculate the
     * controller output.
     *
     * @param error The magnitude of the current measurement
     * @param dt Time interval (dT) since the previous input
     * @return The output from the PID controller
     */
    T Input(const T& error, float dt)
    {
        output_b_ = bias_;

        /*
         * Proportional output
         */
        output_p_ = error * kp_;

        /*
         * Integral output
         */
        output_i_ = output_i_ + 0.5f * ki_ * dt * (error + last_error_);

        /*
         * Anti-windup dynamic clamping of the integral component
         */
        float dynamicLimitMin, dynamicLimitMax;
        if (output_p_ < output_max_)
            dynamicLimitMax = output_max_ - output_p_;
        else
            dynamicLimitMax = 0.0f;

        if (output_p_ > -output_max_)
            dynamicLimitMin = -output_max_ - output_p_;
        else
            dynamicLimitMin = 0.0f;

        /*
         * Clamp the integral output
         */
        if (output_i_ > dynamicLimitMax)
            output_i_ = dynamicLimitMax;
        if (output_i_ < dynamicLimitMin)
            output_i_ = dynamicLimitMin;

        /*
         * Calculate the differential output and then run it through low pass filter:
         * (error - last_error_) * kd_ / dt
         * And then pass it through the low pass filter:
         * filtered = filtered + (input - filtered) * alpha;
         */
        output_d_ = output_d_ + (((error - last_error_) * kd_ / dt) - output_d_) * alpha_d_;
        last_error_ = error;
        return Output();
    }


    T Error() const
    {
        return last_error_;
    }

    /** Return the current output calculated during the last @ref Input call
     *
     * @return Return the cached output
     */
    T Output() const
    {
        T output = output_b_ + output_p_ + output_i_ + output_d_;
        if (output > output_max_)
            return output_max_;
        else if (output < -output_max_)
            return -output_max_;
        return output;
    }

    /** Return the proportional component of the PID controller output
     *
     * @return Proportional output
     */
    T OutputP() const
    {
        return output_p_;
    }

    /** Return the integral component of the PID controller output
     *
     * @return Integral output
     */
    T OutputI() const
    {
        return output_i_;
    }

    /** Return the differential component of the PID controller output
     *
     * @return Differential output
     */
    T OutputD() const
    {
        return output_d_;
    }

    /** Return the current bias. Bias can be set with @ref SetBias method.
     *
     * The bias is a constant value and it doesn't depend on the
     * input or PID gains. The bias value is added to the
     * calculated PID output (which depends both on the input and the gains)
     *
     * @return The current controller bias
     */
    T OutputB() const
    {
        return output_b_;
    }

    /** Set the maximum allowed output.
     *
     * @param output_max max output from the PID controller.
     */
    void SetMaxOutput(const T &output_max)
    {
        output_max_ = (output_max < 0) ? -output_max : output_max;
    }

    /** Return the output limit
     *
     * @return Output limit
     */
    T GetMaxOutput() const
    {
        return output_max_;
    }

    /** Set the PID controller output bias.
     *
     * This bias will be added to the variable PID output.
     *
     * @param bias The desired PID controller bias
     */
    void SetBias(const T &bias)
    {
        bias_ = bias;
    }

    /** Get the current bias
     *
     * @return Current bias
     * @see SetBias
     */
    T GetBias() const
    {
        return bias_;
    }

    /** Get the Low Pass filter coefficient for the differential term
     *
     * @return Low Pass filter coefficient for the differential term
     */
    float GetAlpahD() const
    {
        return alpha_d_;
    }

    /** Set the proportional gain of the PID controller
     *
     * @param kp Proportional gain
     */
    void SetGainP(float kp)
    {
        kp_ = kp;
    }

    /** Set the integral gain of the PID controller
     *
     * @param ki Integral gain
     */
    void SetGainI(float ki)
    {
        ki_ = ki;
    }

    /** Set the differential gain of the PID controller
     *
     * @param kd Differential gain
     */
    void SetGainD(float kd)
    {
        kd_ = kd;
    }

    /** Set the low pass filter alpha for the differential term
     *
     * @param alpha_d Low Pass filter coefficient
     */
    void SetAlphaD(float alpha_d)
    {
        alpha_d_ = alpha_d;
    }

    /** Reset the controller output
     *
     * Please note this call doesn't reset the bias.
     * After this call the PID output would be 0, unless
     * the PID controller is biased. If there is bias set
     * the output will be the bias value.
     *
     */
    void Reset()
    {
        last_error_ = 0;
        output_p_ = 0;
        output_i_ = 0;
        output_d_ = 0;
    }

public:
    float kp_;              /**< Proportional gain */
    float ki_;              /**< Integral gain */
    float kd_;              /**< Differential gain */
    float alpha_d_;         /**< Alpha coefficient for the D-lowpass filter. */
    T output_max_;          /**< Output limit */
    T bias_;                /**< PID controller bias */
    T last_error_;          /**< Cached error value from the @ref InputError method call */
    T output_p_;            /**< Proportional output */
    T output_i_;            /**< Integral output */
    T output_d_;            /**< Differential output */
    T output_b_;            /**< Bias output */
};

#endif /* _PIDCONTROLLER_H_ */
