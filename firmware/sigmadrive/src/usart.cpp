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
		uint32_t hwflowctrl,
		uint32_t timeout
		)
	: USARTx_(usart_device)
	, DMAx_(dma_device)
	, tx_stream_(tx_stream)
	, rx_stream_(rx_stream)
{
	for (auto& pin : data_pins)
		pin.init();

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
		InterruptManager::instance().callback(USART1_IRQn, &USART::usart_irq_handler, this);
		NVIC_SetPriority(USART1_IRQn, 0);
		NVIC_EnableIRQ(USART1_IRQn);
		g_uart[1] = this;
	} else if (usart_device == USART2) {
		__USART2_CLK_ENABLE();
		InterruptManager::instance().callback(USART2_IRQn, &USART::usart_irq_handler, this);
		NVIC_SetPriority(USART2_IRQn, 0);
		NVIC_EnableIRQ(USART2_IRQn);
		g_uart[2] = this;
	} else if (usart_device == USART6) {
		__USART6_CLK_ENABLE();
		InterruptManager::instance().callback(USART6_IRQn, &USART::usart_irq_handler, this);
		NVIC_SetPriority(USART6_IRQn, 0);
		NVIC_EnableIRQ(USART6_IRQn);
		g_uart[6] = this;
	}
	if (LL_USART_Init(USARTx_, &Init) != SUCCESS) {
		throw std::runtime_error("Failed to init UART");
	}

	/*
	 * Enable the DMA device
	 */
	if (DMAx_ == DMA1)
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	else if (DMAx_ == DMA2)
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	/*
	 * Configure NVIC for DMA transfer complete/error interrupts
	 */
	InterruptManager::instance().callback(DMA2_Stream7_IRQn, &USART::dma_irq_handler, this);
	NVIC_SetPriority(DMA2_Stream7_IRQn, 0);
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);

	/*
	 * Configure the DMA TX stream
	 */
	LL_DMA_ConfigTransfer(DMAx_, tx_stream, LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetChannelSelection(DMAx_, tx_stream, dma_channel);

	enable();
}

USART::~USART()
{
	disable();
}

void USART::usart_irq_handler(void)
{
#if defined(DEBUG)
	__DEBUG_BKPT();
#endif
	while (1)
	{
	}
}

void USART::dma_irq_handler(void)
{
	if (LL_DMA_IsActiveFlag_DME7(DMA2)) {
		LL_DMA_ClearFlag_DME7(DMA2);

	}
	if (LL_DMA_IsActiveFlag_FE7(DMA2)) {
		LL_DMA_ClearFlag_FE7(DMA2);

	}
	if (LL_DMA_IsActiveFlag_HT7(DMA2)) {
		LL_DMA_ClearFlag_HT7(DMA2);

	}
	if (LL_DMA_IsActiveFlag_TC7(DMA2)) {
		LL_DMA_ClearFlag_TC7(DMA2);
		transmitting_ = false;
	}
	if (LL_DMA_IsActiveFlag_TE7(DMA2)) {
		LL_DMA_ClearFlag_TE7(DMA2);
	}
}


ssize_t USART::write(const char* buf, size_t nbytes)
{
	size_t i = 0;

	for (i = 0; i < nbytes; i++) {
		while(!LL_USART_IsActiveFlag_TXE(USARTx_))
			;
		LL_USART_TransmitData8(USARTx_, buf[i]);
	}

	return nbytes;
}


ssize_t USART::write_dma(const char* buf, size_t nbytes)
{
//	while (LL_DMA_IsEnabledStream(DMAx_, tx_stream_));
//	LL_DMA_ClearFlag_TC7(DMA2);

	transmitting_ = true;
	LL_DMA_EnableIT_TC(DMAx_, tx_stream_);
	LL_DMA_ConfigAddresses(DMAx_, tx_stream_, (uint32_t)buf, LL_USART_DMA_GetRegAddr(USARTx_), LL_DMA_GetDataTransferDirection(DMAx_, tx_stream_));
	LL_DMA_SetDataLength(DMAx_, tx_stream_, nbytes);
	LL_USART_EnableDMAReq_TX(USARTx_);
	LL_DMA_EnableStream(DMAx_, tx_stream_);

	while (transmitting_);

	return nbytes;
}

