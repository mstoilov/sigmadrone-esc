/*
 * usartde.h
 *
 *  Created on: Mar 23, 2019
 *      Author: mstoilov
 */

#ifndef USARTDE_H_
#define USARTDE_H_

#include "usart.h"
#include "digitalout.h"

class USARTDE : public USART{
public:
	using base = USART;
	USARTDE(PinName de_pinname,
			const std::vector<GPIOPin>& data_pins = {},
			uint32_t baudrate = 230400,
			USART_TypeDef* usart_device = USART1,
			DMA_TypeDef *dma_device = DMA2,
			uint32_t tx_stream = LL_DMA_STREAM_7,
			uint32_t rx_stream = LL_DMA_STREAM_5,
			uint32_t dma_channel = LL_DMA_CHANNEL_4,
			uint32_t hwflowctrl = LL_USART_HWCONTROL_NONE,
			uint32_t irq_priority = 0);

	virtual ~USARTDE();
	inline void EnableDE() { de_pin_.Write(1); }
	inline void DisableDE() { de_pin_.Write(0); }

protected:
	virtual void OnTxBegin(void) override;
	virtual void OnTxComplete(void) override;

public:
	DigitalOut de_pin_;
};

#endif /* USARTDE_H_ */
