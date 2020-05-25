
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
     * @param decay Decay rate
     * @param output_i_max Maximum integral output limit
     * @param bias Output bias
     */
    PidController(float kp = 0, float ki = 0, float kd = 0, float alpha_d = 1.0, float decay = 0, const T &output_i_max = 0, const T &bias = 0)
        : kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , alpha_d_(alpha_d)
        , decay_(decay)
        , output_i_max_(output_i_max)
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
     * @param setpoint The magnitude of the controlled setpoint
     * @param measurement The magnitude of the current measurement
     * @param dt Time interval (dT) since the previous input
     * @return The output from the PID controller
     */
    T Input(const T& setpoint, const T& measurement, float dt)
    {
        T error = setpoint - measurement;
        return InputError(error, measurement, dt);
    }

    T InputError(const T& error, const T& measurement, float dt)
    {
        output_b_ = bias_;
        output_p_ = error * kp_;
        if (decay_ > 0) {
            float decay = (1.0f - decay_ * dt);
            output_i_ *= decay;
        }
        output_i_ += 0.5f * (error + last_error_) * ki_ * dt;
        if (output_i_max_ && output_i_ > output_i_max_)
            output_i_ = output_i_max_;
        if (output_i_max_ && output_i_ < -output_i_max_)
            output_i_ = -output_i_max_;
        /*
         * Calculate the differential output, by differentiating measurement instead of the error:
         * (measurement - last_measurement_) * kd_ / dt
         * And then pass it through the low pass filter:
         * filtered = filtered + (input - filtered) * alpha;
         */
        output_d_ = output_d_ + (((measurement - last_measurement_) * kd_ / dt) - output_d_) * alpha_d_;
        last_error_ = error;
        last_measurement_ = measurement;
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
        return output_b_ + output_p_ + output_i_ + output_d_;
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

    /** Set the maximum allow integral component of the output.
     *
     * Setting the max integral output is used in cases where
     * the integral component can wind up over time. PID controller
     * wind up is a common problem and this is just one of the
     * possible solutions.
     *
     * @param output_max max integral component.
     */
    void SetMaxIntegralOutput(const T &output_max)
    {
        output_i_max_ = output_max;
    }

    /** Return the current integral component limit
     *
     * @return Integral component limit
     */
    T GetMaxIntegralOutput() const
    {
        return output_i_max_;
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

    /** Set the decay rate of the currently accumulated integral output
     *
     * The integral output is adusted using the following formula:
     * output_i_ *= (1.0f - decay_ * dt);
     *
     * @param decay The decay rate
     */
    void SetDecayRate(float decay)
    {
        decay_ = decay;
    }

    /** Get the current decay rate for the integral output
     *
     * @return Deacay rate
     */
    float GetDecayRate() const
    {
        return decay_;
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
    float decay_;           /**< Decay rate */
    T output_i_max_;        /**< Integral output limit */
    T bias_;                /**< PID controller bias */
    T last_error_;          /**< Cached error value from the @ref InputError method call */
    T last_measurement_;    /**< Cached measurement from the @ref InputError method call */
    T output_p_;            /**< Proportional output */
    T output_i_;            /**< Integral output */
    T output_d_;            /**< Differential output */
    T output_b_;            /**< Bias output */
};

#endif /* _PIDCONTROLLER_H_ */
