#include "pwmencoder.h"

/*
#define UH_PIN		PA_8
#define UL_PIN		PB_13

#define VH_PIN		PA_9
#define VL_PIN		PB_14

#define WH_PIN		PA_10
#define WL_PIN		PB_1
*/

#define UH_PIN		PA_8
#define UL_PIN		PA_7

#define VH_PIN		PA_9
#define VL_PIN		PB_0

#define WH_PIN		PA_10
#define WL_PIN		PB_1

#define TG_PIN		PC_6



#define USE_IHM08M1
PWMEncoder::PWMEncoder(TIM_TypeDef *TIMx, const TimeSpan& pwm_period, const Frequency& system_clock, const std::vector<GPIOPin>& output_pins)
	: Timer(TIMx, pwm_period, system_clock, output_pins)
	, period_(pwm_period)
	, state_(0UL)
#ifdef USE_IHM08M1
	, UL_(UL_PIN, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullDown, DigitalOut::ActiveLow, 0)
	, VL_(VL_PIN, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullDown, DigitalOut::ActiveLow, 0)
	, WL_(WL_PIN, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullDown, DigitalOut::ActiveLow, 0)
	, TG_(TG_PIN, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullNone, DigitalOut::ActiveHigh, 0)
#else
	, UL_(UL_PIN, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullDown, DigitalOut::ActiveDefault, 0)
	, VL_(VL_PIN, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullDown, DigitalOut::ActiveDefault, 0)
	, WL_(WL_PIN, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullDown, DigitalOut::ActiveDefault, 0)
	, TG_(TG_PIN, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullNone, DigitalOut::ActiveHigh, 0)
#endif
{
	SetAutoReloadPeriod(pwm_period);

	SetOCMode(CH1, PWM1);
	SetOCMode(CH2, PWM1);
	SetOCMode(CH3, PWM1);

#ifdef USE_IHM08M1
	SetOCPolarity(CH1, High);
	SetOCPolarity(CH2, High);
	SetOCPolarity(CH3, High);
#else
	SetOCPolarity(CH1, Low);
	SetOCPolarity(CH2, Low);
	SetOCPolarity(CH3, Low);
#endif

	EnableChannel(CH1);
	EnableChannel(CH2);
	EnableChannel(CH3);

	EnableOutputs();
	GenerateEvent(EventUpdate);

}

PWMEncoder::~PWMEncoder()
{
	Stop();
}

void PWMEncoder::Start()
{
	EnableInterrupt(InterruptUpdate);
	base::Start();
}

void PWMEncoder::Stop()
{
	DisableInterrupt(InterruptUpdate);
	base::Stop();

	SetOCMode(CH1, Inactive);
	SetOCMode(CH2, Inactive);
	SetOCMode(CH3, Inactive);
	UL_.Off();
	VL_.Off();
	WL_.Off();
}

void PWMEncoder::Toggle()
{
	if (IsEnabledCounter())
		Stop();
	else
		Start();
}

void PWMEncoder::SetDutyCycle(float duty)
{
	uint32_t ocval = GetAutoReloadValue() * duty;

	if (ocval > GetAutoReloadValue())
		ocval = GetAutoReloadValue();
	SetOCValue(CH1, GetAutoReloadValue() * duty);
	SetOCValue(CH2, GetAutoReloadValue() * duty);
	SetOCValue(CH3, GetAutoReloadValue() * duty);
}

void PWMEncoder::SetDutyPeriod(const TimeSpan& period)
{
	SetOCPeriod(CH1, period);
	SetOCPeriod(CH2, period);
	SetOCPeriod(CH3, period);
}

void PWMEncoder::DumbDriving()
{
	if (GetStatus() & FlagUpdate) {
		state_ = (state_ + 1) % 6;
		switch(state_) {
		case 0:
			TG_.On();
			SetOCMode(CH1, PWM1);
			SetOCMode(CH2, Inactive);
			SetOCMode(CH3, Inactive);
			UL_.Off();
			VL_.Off();
			WL_.On();
			break;
		case 1:
			TG_.Off();
			SetOCMode(CH1, Inactive);
			SetOCMode(CH2, PWM1);
			SetOCMode(CH3, Inactive);
			UL_.Off();
			VL_.Off();
			WL_.On();
			break;
		case 2:
			SetOCMode(CH1, Inactive);
			SetOCMode(CH2, PWM1);
			SetOCMode(CH3, Inactive);
			UL_.On();
			VL_.Off();
			WL_.Off();
			break;
		case 3:
			SetOCMode(CH1, Inactive);
			SetOCMode(CH2, Inactive);
			SetOCMode(CH3, PWM1);
			UL_.On();
			VL_.Off();
			WL_.Off();
			break;
		case 4:
			SetOCMode(CH1, Inactive);
			SetOCMode(CH2, Inactive);
			SetOCMode(CH3, PWM1);
			UL_.Off();
			VL_.On();
			WL_.Off();
			break;
		case 5:
			SetOCMode(CH1, PWM1);
			SetOCMode(CH2, Inactive);
			SetOCMode(CH3, Inactive);
			UL_.Off();
			VL_.On();
			WL_.Off();
			break;

		}
	}
}

void PWMEncoder::HandleUpdate()
{
	DumbDriving();
}
