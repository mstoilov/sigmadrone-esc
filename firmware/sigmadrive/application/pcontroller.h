
#ifndef _PCONTROLLER_H_
#define _PCONTROLLER_H_

/** P controller implementation
 *
 * This is a straight forward implementation of PI controller
 * functionality.
 *
 */
template<typename T>
class PController {
public:
    /** Constructor
     *
     * @param kp Proportional gain
     * @param output_max Maximum integral output limit
     */
    PController(float kp = 0, float output_max = 0)
        : kp_(kp)
        , output_max_((output_max < 0) ? -output_max : output_max)
        , output_p_(0)
    {
    }

    ~PController()
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
        /*
         * Proportional output
         */
        output_p_ = error * kp_;

        return Output();
    }

    /** Return the current output calculated during the last @ref Input call
     *
     * @return Return the cached output
     */
    T Output() const
    {
        T output = output_p_;
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

    /** Set the proportional gain of the PID controller
     *
     * @param kp Proportional gain
     */
    void SetGainP(float kp)
    {
        kp_ = kp;
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
        output_p_ = 0;
    }

public:
    float kp_;              /**< Proportional gain */
    T output_max_;          /**< Output limit */
    T output_p_;            /**< Proportional output */
};

#endif /* _PIDCONTROLLER_H_ */
