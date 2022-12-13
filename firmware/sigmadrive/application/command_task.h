#ifndef _COMMAND_TASK_H_
#define _COMMAND_TASK_H_

#include "ryno/rycpu.h"
#include "ryno/stdfunc.h"
#include "ryno/ryno.h"

#ifdef __cplusplus
extern "C" {
#endif

extern ryno::RunTime rt;
void RunRynoCommandTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* _COMMAND_TASK_H_ */
