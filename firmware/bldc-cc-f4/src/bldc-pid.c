#include "bldc-pid.h"

void bldc_pid_init(BldcPID *pid, int32_t uP, int32_t uI, int32_t uD, int32_t uLeak)
{
	pid->uP = uP;
	pid->uI = uI;
	pid->uD = uD;
	pid->uLeak = uLeak;
	pid->lastError = 0;
}

void bldc_pid_set(BldcPID *pid, int32_t error, int32_t uSec)
{
	pid->accumError += error;
	int32_t derivative = pid->uD * (error - pid->lastError) / uSec;
	int32_t proportional = error * pid->uP;
	int32_t integral = pid->accumError * pid->uI;

	pid->accumError = pid->accumError * (1000000 - pid->uLeak) / 1000000;
	pid->ctrlValue = (integral + proportional) / 1000000 + derivative;
}

int32_t bldc_pid_get(BldcPID *pid)
{
	return pid->ctrlValue;
}
