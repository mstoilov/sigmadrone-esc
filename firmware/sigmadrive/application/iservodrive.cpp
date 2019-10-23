#include "iservodrive.h"

IServoDrive::IServoDrive()
{

}

IServoDrive::~IServoDrive()
{

}


void IServoDrive::SetEncoder(IEncoder *encoder)
{
	encoder_ = encoder;
}

IEncoder* IServoDrive::GetEncoder()
{
	return encoder_;
}

void IServoDrive::SetPwmGenerator(IPwmGenerator* pwm)
{
	pwm_ = pwm;
}

IPwmGenerator* IServoDrive::GetPwmGenerator()
{
	return pwm_;
}
