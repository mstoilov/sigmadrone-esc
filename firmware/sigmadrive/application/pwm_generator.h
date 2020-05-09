/*
 * pwm_generator.h
 *
 *  Created on: Oct 9, 2019
 *      Author: mstoilov
 */

#ifndef _PWM_GENERATOR_H_
#define _PWM_GENERATOR_H_

#define USE_HAL_TIM_REGISTER_CALLBACKS 1

#include "stm32f745xx.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_tim.h"
#include "stm32f7xx_ll_tim.h"
#include "ipwmgenerator.h"

/** Controlls the PWM hardware component
 *
 */
class PwmGenerator : public IPwmGenerator {
public:
    PwmGenerator();
    ~PwmGenerator();
    void Attach(TIM_HandleTypeDef* htim);
    void EnableOutputs(bool enable)	    { enable ? LL_TIM_EnableAllOutputs(htim_->Instance) : LL_TIM_DisableAllOutputs(htim_->Instance); }
    bool IsEnalbedOutputs()             { return LL_TIM_IsEnabledAllOutputs(htim_->Instance) ? true : false; }
    void EnableCounter(bool enable)     { enable ? LL_TIM_EnableCounter(htim_->Instance) : LL_TIM_DisableCounter(htim_->Instance); }
    void LoadSafeTimings();

    virtual void Start() override;
    virtual void Stop() override;
    virtual uint32_t GetCounterDirection() override;
    virtual bool IsStarted() override;
    virtual uint32_t GetPeriod() override;
    virtual void SetPeriod(uint32_t period) override;
    virtual void GetTimings(uint32_t* values, size_t count) override;
    virtual void SetTimings(const uint32_t* values, size_t count) override;
    virtual uint32_t GetTiming(uint32_t channel) override;
    virtual void SetTiming(uint32_t channel, uint32_t value) override;


protected:
    TIM_HandleTypeDef* htim_;
};

#endif /* _PWM_GENERATOR_H_ */
