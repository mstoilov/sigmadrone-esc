#include <string.h>
#include "cortexm/ExceptionHandlers.h"
#include "usart.h"

USART::USART(const std::vector<GPIOPin>& data_pins,
		uint32_t baudrate,
		USART_TypeDef* usart_device,
		DMA_TypeDef *dma_device,
		uint32_t tx_stream,
		uint32_t rx_stream,
		uint32_t dma_channel,
		uint32_t hwflowctrl
		)
	: USARTx_(usart_device)
	, dma_tx_(dma_device, tx_stream, dma_channel, LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE)
	, dma_rx_(dma_device, rx_stream, dma_channel, LL_DMA_MODE_CIRCULAR | LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE)
{
	for (auto& pin : data_pins)
		pin.Init();

	/*##-1- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART1 configured as follow:
	 - Word Length = 8 Bits
	 - Stop Bit = One Stop bit
	 - Parity = None
	 - BaudRate = 230400 baud
	 - Hardware flow control disabled (RTS and CTS signals) */
	LL_USART_InitTypeDef Init;
	Init.BaudRate = baudrate;
	Init.DataWidth = LL_USART_DATAWIDTH_8B;
	Init.StopBits = LL_USART_STOPBITS_1;
	Init.Parity = LL_USART_PARITY_NONE;
	Init.HardwareFlowControl = hwflowctrl;
	Init.OverSampling = LL_USART_OVERSAMPLING_8;
	Init.TransferDirection = LL_USART_DIRECTION_TX_RX;

	if (usart_device == USART1) {
		__USART1_CLK_ENABLE();
		InterruptManager::instance().Callback_EnableIRQ(USART1_IRQn, 0, &USART::IrqHandlerUSART, this);
	} else if (usart_device == USART2) {
		__USART2_CLK_ENABLE();
		InterruptManager::instance().Callback_EnableIRQ(USART2_IRQn, 0, &USART::IrqHandlerUSART, this);
	} else if (usart_device == USART6) {
		__USART6_CLK_ENABLE();
		InterruptManager::instance().Callback_EnableIRQ(USART6_IRQn, 0, &USART::IrqHandlerUSART, this);
	}
	if (LL_USART_Init(USARTx_, &Init) != SUCCESS) {
		throw std::runtime_error("Failed to init UART");
	}

	LL_USART_EnableIT_RXNE(USARTx_);
	dma_tx_.Callback_TC(this, &USART::CallbackTX_DmaTC);
	Enable();
	StartDmaRx();
}

USART::~USART()
{
	Disable();
}

void USART::IrqHandlerUSART(void)
{
	LL_USART_ClearFlag_RXNE(USARTx_);
	LL_USART_ClearFlag_ORE(USARTx_);
}

void USART::StartDmaRx()
{
	dma_rx_.ConfigAddresses(LL_USART_DMA_GetRegAddr(USARTx_), (uint32_t)input_queue_.get_data_ptr(), dma_rx_.GetDataTransferDirection());
	dma_rx_.SetDataLength(input_queue_.capacity());
	EnableDMAReq_RX();
	dma_rx_.Enable();
}

void USART::CallbackTX_DmaTC(void)
{
	output_queue_.read_update(outputNDT);
	outputNDT = 0UL;
	if (!dma_tx_.IsEnabled())
		StartDmaTx();
}

void USART::StartDmaTx()
{
	outputNDT = output_queue_.read_size();
	if (outputNDT > 0) {
		dma_tx_.EnableIT_TC();
		dma_tx_.ConfigAddresses((uint32_t)output_queue_.get_read_ptr(), LL_USART_DMA_GetRegAddr(USARTx_), dma_tx_.GetDataTransferDirection());
		dma_tx_.SetDataLength(outputNDT);
		EnableDMAReq_TX();
		dma_tx_.Enable();
	}
}


ssize_t USART::Write(const char* buf, size_t nbytes)
{
	size_t i = 0;

	for (i = 0; i < nbytes; i++) {
		while(!LL_USART_IsActiveFlag_TXE(USARTx_))
			;
		LL_USART_TransmitData8(USARTx_, buf[i]);
	}

	return nbytes;
}


ssize_t USART::WriteDMA(const char* buf, size_t nbytes)
{
	size_t i = 0;

	while (!output_queue_.space())
		;
	for (i = 0; i < nbytes && output_queue_.space(); i++) {
		output_queue_.push(buf[i]);
	}
	__disable_irq();
	if (i > 0 && !dma_tx_.IsEnabled()) {
		StartDmaTx();
	}
	__enable_irq();
	return i;
}


ssize_t USART::ReadDMA(char* buf, size_t nbytes)
{
	/*
	 * Update the write ptr according to the number of data elements transfered
	 * from the DMA.
	 */
	input_queue_.reset_wp(input_queue_.capacity() - dma_rx_.GetDataLength());

	size_t i = std::min(nbytes, input_queue_.read_size());
	memcpy(buf, input_queue_.get_read_ptr(), i);
	input_queue_.read_update(i);
	return i;
}
