/*
 * usartde.cpp
 *
 *  Created on: Mar 23, 2019
 *      Author: mstoilov
 */

#include "usartde.h"

USARTDE::USARTDE(PinName de_pinname,
		const std::vector<GPIOPin>& data_pins,
		uint32_t baudrate,
		USART_TypeDef* usart_device,
		DMA_TypeDef *dma_device,
		uint32_t tx_stream,
		uint32_t rx_stream,
		uint32_t dma_channel,
		uint32_t hwflowctrl,
		uint32_t irq_priority)
	: USART(data_pins, baudrate, usart_device, dma_device, tx_stream, rx_stream, dma_channel, hwflowctrl, irq_priority)
	, de_pin_(de_pinname, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullDown, DigitalOut::ActiveDefault, 0)
{
	de_pin_.Write(0);
}


USARTDE::~USARTDE()
{

}

void USARTDE::OnTxComplete(void)
{
	DisableDE();
}

void USARTDE::OnTxBegin()
{
	EnableDE();
}
