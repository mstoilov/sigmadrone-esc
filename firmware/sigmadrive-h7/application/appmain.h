#ifndef _APPMAIN_H_
#define _APPMAIN_H_


#ifdef __cplusplus
extern "C" {
#endif

int application_main();
int8_t cdc_recv_data(void *dev, char* buf, size_t nsize);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "uart.h"

extern Uart uart1;

#endif

#endif
