#ifndef _BLDC_PID_H_
#define _BLDC_PID_H_

#include "bldc-include.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct BldcPID_ {
	int32_t uP;
	int32_t uI;
	int32_t uD;
	int32_t uLeak;
	int32_t lastError;
	int32_t accumError;
	int32_t ctrlValue;
} BldcPID;


void bldc_pid_init(BldcPID *pid, int32_t uP, int32_t uI, int32_t uD, int32_t uLeak);
void bldc_pid_set(BldcPID *pid, int32_t error, int32_t uSec);
int32_t bldc_pid_get(BldcPID *pid);

#ifdef __cplusplus
}
#endif

#endif
