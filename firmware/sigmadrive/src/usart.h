#ifndef _USART_H_
#define _USART_H_

#include <stdint.h>
#include <vector>
#include <string>
#include <stdexcept>

#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f411xe.h"

#include "gpiopin.h"
#include "pinnames.h"
#include "dma.h"
#include "interruptmanager.h"

class USART {
public:
	USART(const std::vector<GPIOPin>& data_pins = {},
			uint32_t baudrate = 230400,
			USART_TypeDef* usart_device = USART1,
			DMA_TypeDef *dma_device = DMA2,
			uint32_t tx_stream = LL_DMA_STREAM_7,
			uint32_t rx_stream = LL_DMA_STREAM_5,
			uint32_t dma_channel = LL_DMA_CHANNEL_4,
			uint32_t hwflowctrl = LL_USART_HWCONTROL_NONE
			);
	virtual ~USART();

	void Enable(void)			{ LL_USART_Enable(USARTx_); }
	void Disable(void)			{ LL_USART_Disable(USARTx_); }
	void EnableDMAReq_TX(void)	{ LL_USART_EnableDMAReq_TX(USARTx_); }
	void EnableDMAReq_RX(void)	{ LL_USART_EnableDMAReq_RX(USARTx_); }
	void DisableDMAReq_TX(void)	{ LL_USART_DisableDMAReq_TX(USARTx_); }
	void DisableDMAReq_RX(void)	{ LL_USART_DisableDMAReq_RX(USARTx_); }

	ssize_t Write(const char* buf, size_t nbytes);
	ssize_t WriteDMA(const char* buf, size_t nbytes);

private:
	void IrqHandlerUSART(void);
	void CallbackTX_DmaTC(void);

public:
	USART_TypeDef* USARTx_;
	Dma dma_tx_;
	Dma dma_rx_;
	bool transmitting_ = 0;


};

#endif /* _USART_H_ */
