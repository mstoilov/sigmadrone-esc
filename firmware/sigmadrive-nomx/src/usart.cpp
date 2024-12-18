#include <string.h>
#include <cassert>
#include "cortexm/ExceptionHandlers.h"
#include "usart.h"

USART::USART(const std::vector<GPIOPin>& data_pins,
		uint32_t baudrate,
		USART_TypeDef* usart_device,
		DMA_TypeDef *dma_device,
		uint32_t tx_stream,
		uint32_t rx_stream,
		uint32_t dma_channel,
		uint32_t hwflowctrl,
		uint32_t irq_priority,
		uint32_t dma_irq_priority
		)
	: USARTx_(usart_device)
	, dma_tx_(dma_device, tx_stream, dma_channel, LL_DMA_MODE_NORMAL | LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE, dma_irq_priority)
	, dma_rx_(dma_device, rx_stream, dma_channel, LL_DMA_MODE_CIRCULAR | LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_HIGH | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE, dma_irq_priority)
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
		InterruptManager::instance().Callback_EnableIRQ(USART1_IRQn, irq_priority, &USART::IrqHandlerUSART, this);
	} else if (usart_device == USART2) {
		__USART2_CLK_ENABLE();
		InterruptManager::instance().Callback_EnableIRQ(USART2_IRQn, irq_priority, &USART::IrqHandlerUSART, this);
	} else if (usart_device == USART3) {
		__USART3_CLK_ENABLE();
		InterruptManager::instance().Callback_EnableIRQ(USART3_IRQn, irq_priority, &USART::IrqHandlerUSART, this);
	} else if (usart_device == USART6) {
		__USART6_CLK_ENABLE();
		InterruptManager::instance().Callback_EnableIRQ(USART6_IRQn, irq_priority, &USART::IrqHandlerUSART, this);
	}
	if (LL_USART_Init(USARTx_, &Init) != SUCCESS) {
		throw std::runtime_error("Failed to init UART");
	}

	//LL_USART_EnableIT_TC(USARTx_);
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
	if (LL_USART_IsActiveFlag_TC(USARTx_)) {
		LL_USART_ClearFlag_TC(USARTx_);
		OnTxComplete();
	}

}

void USART::StartDmaRx()
{
	dma_rx_.ConfigAddresses(LL_USART_DMA_GetRegAddr(USARTx_, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t)input_queue_.get_data_ptr(), dma_rx_.GetDataTransferDirection());
	dma_rx_.SetDataLength(input_queue_.capacity());
	EnableDMAReq_RX();
	dma_rx_.Enable();
}

void USART::CallbackTX_DmaTC(void)
{
	output_queue_.read_update(outputNDT_);
	outputNDT_ = 0UL;
	size_t nbytes = output_queue_.read_size();
	if (nbytes && !dma_tx_.IsEnabled())
		StartDmaTx(nbytes);
}

void USART::StartDmaTx(size_t nbytes)
{
	outputNDT_ = nbytes;
	if (outputNDT_ > 0) {
		dma_tx_.EnableIT_TC();
		dma_tx_.ConfigAddresses((uint32_t)output_queue_.get_read_ptr(), LL_USART_DMA_GetRegAddr(USARTx_, LL_USART_DMA_REG_DATA_TRANSMIT), dma_tx_.GetDataTransferDirection());
		dma_tx_.SetDataLength(outputNDT_);
		EnableDMAReq_TX();
		dma_tx_.Enable();
	}
}


ssize_t USART::Write(const char* buf, size_t nbytes)
{
	size_t i = 0;

	LL_USART_ClearFlag_TC(USARTx_);
	OnTxBegin();

	for (i = 0; i < nbytes; i++) {
		while(!LL_USART_IsActiveFlag_TXE(USARTx_))
			;
		LL_USART_TransmitData8(USARTx_, buf[i]);
	}

	while (!LL_USART_IsActiveFlag_TC(USARTx_)) {
		;
	}

	OnTxComplete();
	LL_USART_ClearFlag_TC(USARTx_);

	return nbytes;
}

ssize_t USART::WriteDMA(const char* buf, size_t nbytes)
{
	size_t i = 0;
	uint32_t vector = __get_xPSR() & 0xFF;

	if (vector && !output_queue_.space()) {
		/*
		 * If this is called from interrupt and the queue is out of space
		 * just drop the write request.
		 */
		return nbytes;
	}

	while (!output_queue_.space())
		;
	for (i = 0; i < nbytes && output_queue_.space(); i++) {
		output_queue_.push(buf[i]);
	}
	__disable_irq();
	if (i > 0 && !dma_tx_.IsEnabled()) {
		OnTxBegin();
		StartDmaTx(output_queue_.read_size());
	}
	__enable_irq();
	return i;
}

ssize_t USART::ReadDMAOrBlock(char* buf, size_t nbytes)
{
	ssize_t ret = 0;
	while ((ret = GetRxSize()) == 0)
		;
	return ReadDMA(buf, nbytes);
}

size_t USART::GetRxSize()
{
	/*
	 * Update the write ptr according to the number of data elements transfered
	 * from the DMA.
	 */
	input_queue_.reset_wp(input_queue_.capacity() - dma_rx_.GetDataLength());
	return input_queue_.read_size();
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

std::string USART::Read()
{
	ssize_t read_size = 0;
	while ((read_size = GetRxSize()) == 0)
		;
	std::string ret(input_queue_.get_read_ptr(), read_size);
	input_queue_.read_update(read_size);
	return ret;
}

void USART::Write(const std::string& str)
{
	for (size_t ret = 0, i = 0, n = str.size(); n > 0; n -= ret, i += ret) {
		ret = WriteDMA(str.c_str() + i, n);
	}
}

bool USART::EnableDEMode(
		uint32_t assertion_time,
		uint32_t deassertion_time)
{
	assert(assertion_time < 32);
	assert(deassertion_time < 32);
	if (IS_UART_DRIVER_ENABLE_INSTANCE(USARTx_)) {
		bool was_usart_enabled = IsEnabled();
		Disable();
		LL_USART_SetDESignalPolarity(USARTx_, LL_USART_DE_POLARITY_HIGH);
		LL_USART_SetDEAssertionTime(USARTx_, assertion_time);
		LL_USART_SetDEDeassertionTime(USARTx_, deassertion_time);
		LL_USART_EnableDEMode(USARTx_);
		if (was_usart_enabled) {
			Enable();
		}
		assert(IsDEModeEnabled());
	}
	return IsDEModeEnabled();
}

void USART::DisableDEMode()
{
	bool was_usart_enabled = IsEnabled();
	Disable();
	LL_USART_DisableDEMode(USARTx_);
	if (was_usart_enabled) {
		Enable();
	}
	assert(!IsDEModeEnabled());
}

