#ifndef _BLDC_UART_H_
#define _BLDC_UART_H_

#include "bldc-include.h"


#ifdef __cplusplus
extern "C" {
#endif

void bldc_uart_init(USART_TypeDef *USARTx, uint32_t baudrate, uint32_t hwctrl);
void bldc_uart_start(USART_TypeDef *USARTx);
void bldc_uart_stop(USART_TypeDef *USARTx);


#ifdef __cplusplus
}
#endif

#endif
