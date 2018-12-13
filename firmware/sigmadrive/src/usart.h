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
			uint32_t hwflowctrl = LL_USART_HWCONTROL_NONE,
			uint32_t timeout = 250
			);
	virtual ~USART();

	void enable(void)			{ LL_USART_Enable(USARTx_); }
	void disable(void)			{ LL_USART_Disable(USARTx_); }

	ssize_t write(const char* buf, size_t nbytes);
	ssize_t write_dma(const char* buf, size_t nbytes);

private:
	void usart_irq_handler(void);
	void dma_irq_handler(void);

public:
	USART_TypeDef* USARTx_;
	DMA_TypeDef* DMAx_;
	uint32_t tx_stream_;
	uint32_t rx_stream_;
	bool transmitting_ = 0;


};

#endif /* _USART_H_ */
