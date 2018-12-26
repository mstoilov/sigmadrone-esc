#include "cortexm/ExceptionHandlers.h"
#include "usart.h"

static USART* g_uart[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, };


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
	, dma_rx_(dma_device, rx_stream, dma_channel, LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE)
{
	for (auto& pin : data_pins)
		pin.Init();

	/*##-1- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART1 configured as follow:
	 - Word Length = 8 Bits
	 - Stop Bit = One Stop bit
	 - Parity = None
	 - BaudRate = 9600 baud
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
		InterruptManager::instance().Callback(USART1_IRQn, &USART::IrqHandlerUSART, this);
		NVIC_SetPriority(USART1_IRQn, 0);
		NVIC_EnableIRQ(USART1_IRQn);
		g_uart[1] = this;
	} else if (usart_device == USART2) {
		__USART2_CLK_ENABLE();
		InterruptManager::instance().Callback(USART2_IRQn, &USART::IrqHandlerUSART, this);
		NVIC_SetPriority(USART2_IRQn, 0);
		NVIC_EnableIRQ(USART2_IRQn);
		g_uart[2] = this;
	} else if (usart_device == USART6) {
		__USART6_CLK_ENABLE();
		InterruptManager::instance().Callback(USART6_IRQn, &USART::IrqHandlerUSART, this);
		NVIC_SetPriority(USART6_IRQn, 0);
		NVIC_EnableIRQ(USART6_IRQn);
		g_uart[6] = this;
	}
	if (LL_USART_Init(USARTx_, &Init) != SUCCESS) {
		throw std::runtime_error("Failed to init UART");
	}

	LL_USART_EnableIT_RXNE(USARTx_);
	dma_tx_.Callback_TC(this, &USART::CallbackTX_DmaTC);
	dma_rx_.Callback_TC(this, &USART::CallbackRX_DmaTC);
	Enable();
	StartDmaRx();
}

USART::~USART()
{
	Disable();
}

void USART::IrqHandlerUSART(void)
{
	if (LL_USART_IsActiveFlag_RXNE(USARTx_))
		LL_USART_ClearFlag_RXNE(USARTx_);
	if (LL_USART_IsActiveFlag_ORE(USARTx_))
		LL_USART_ClearFlag_ORE(USARTx_);

}

void USART::CallbackTX_DmaTC(void)
{
	if (!dma_tx_.IsEnabled())
		StartDmaTx();
}

void USART::CallbackRX_DmaTC(void)
{
	/*
	 * Re-enable the DMA transfer.
	 */
	StartDmaRx();
}

void USART::StartDmaRx()
{
	input_queue_.reset_wp(0);
	dma_rx_.EnableIT_TC();
	dma_rx_.ConfigAddresses(LL_USART_DMA_GetRegAddr(USARTx_), (uint32_t)input_queue_.get_write_ptr(), dma_rx_.GetDataTransferDirection());
	dma_rx_.SetDataLength(input_queue_.capacity());
	EnableDMAReq_RX();
	dma_rx_.Enable();
}

void USART::StartDmaTx()
{
	size_t i = 0;
	for (i = 0; i < output_buffer_.size() && output_queue_.size() > 0; i++) {
		output_buffer_[i] = output_queue_.front();
		output_queue_.pop();
	}

	if (i > 0) {
		dma_tx_.EnableIT_TC();
		dma_tx_.ConfigAddresses((uint32_t)output_buffer_.data(), LL_USART_DMA_GetRegAddr(USARTx_), dma_tx_.GetDataTransferDirection());
		dma_tx_.SetDataLength(i);
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
	while (!output_queue_.space())
		;
	size_t i = 0;
	for (i = 0; i < nbytes && output_queue_.space(); i++) {
		output_queue_.push(buf[i]);
	}
	if (i > 0 && !dma_tx_.IsEnabled()) {
		StartDmaTx();
	}
	return i;
}


ssize_t USART::ReadDMA(char* buf, size_t nbytes)
{
	size_t i = 0;

	__disable_irq();
	input_queue_.reset_wp(input_queue_.capacity() - dma_rx_.GetDataLength());
	__enable_irq();

	for (i = 0; i < nbytes && input_queue_.size() > 0; i++) {
		buf[i] = input_queue_.front();
		input_queue_.pop();
	}

	return i;
}
