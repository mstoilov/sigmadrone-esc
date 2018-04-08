#include "assert.h"

#include "bldc-usart.h"


void bldc_uart_init(USART_TypeDef *USARTx, uint32_t baudrate, uint32_t hwctrl)
{
	LL_USART_InitTypeDef Init;

	Init.BaudRate = baudrate;
	Init.DataWidth = LL_USART_DATAWIDTH_8B;
	Init.StopBits = LL_USART_STOPBITS_1;
	Init.Parity = LL_USART_PARITY_NONE;
	Init.HardwareFlowControl = hwctrl;
	Init.OverSampling = LL_USART_OVERSAMPLING_8;
	Init.TransferDirection = LL_USART_DIRECTION_TX_RX;

	if (LL_USART_Init(USARTx, &Init) != SUCCESS) {
		assert(0);
	}

}


void bldc_uart_start(USART_TypeDef *USARTx)
{
	LL_USART_Enable(USARTx);
}


void bldc_uart_stop(USART_TypeDef *USARTx)
{
	LL_USART_Disable(USARTx);
}
