#ifndef _USART_H_
#define _USART_H_

#include <stdint.h>
#include <vector>
#include <array>
#include <queue>
#include <string>
#include <stdexcept>

#include "sigmadrive.h"

#include "gpiopin.h"
#include "pinnames.h"
#include "dma.h"
#include "interruptmanager.h"
#include "ring.h"
#include "digitalout.h"

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
			uint32_t irq_priority = 0,
			uint32_t dma_irq_priority = 0);
	virtual ~USART();

	void Enable(void)			{ LL_USART_Enable(USARTx_); }
	void Disable(void)			{ LL_USART_Disable(USARTx_); }
	bool IsEnabled(void)			{ return LL_USART_IsEnabled(USARTx_) ? true : false; }

	ssize_t Write(const char* buf, size_t nbytes);
	ssize_t WriteDMA(const char* buf, size_t nbytes);
	ssize_t ReadDMA(char* buf, size_t nbytes);
	ssize_t ReadDMAOrBlock(char* buf, size_t nbytes);
	size_t GetRxSize();
	std::string Read();
	void Write(const std::string& str);

	// Enables DE mode on the USART. Returns true if DE mode was successfully enabled.
	// Parameters:
	// - assertion_time is the No. of sample time units (1/8 or 1/16 bit duration, depending on the oversampling rate)
	// from the time the DE signal is asserted to the beginning of the start bit; valid range is 0-31
	// - deassertion_time is the No of sample time units from the end of the last stop bit and the de-activation of
	// the DE signal; valid range is 0-31
	bool EnableDEMode(
			uint32_t assertion_time = 8,
			uint32_t deassertion_time = 4);
	void DisableDEMode();
	bool IsDEModeEnabled() { return LL_USART_IsEnabledDEMode(USARTx_);}


private:
	void EnableDMAReq_TX(void)	{ LL_USART_EnableDMAReq_TX(USARTx_); }
	void EnableDMAReq_RX(void)	{ LL_USART_EnableDMAReq_RX(USARTx_); }
	void DisableDMAReq_TX(void)	{ LL_USART_DisableDMAReq_TX(USARTx_); }
	void DisableDMAReq_RX(void)	{ LL_USART_DisableDMAReq_RX(USARTx_); }
	void CallbackTX_DmaTC(void);
	void StartDmaRx();
	void StartDmaTx(size_t nbytes);
	void IrqHandlerUSART(void);

protected:
	virtual void OnTxBegin(void) { };
	virtual void OnTxComplete(void) { };

public:
	Ring<char, 256> output_queue_;
	Ring<char, 2048> input_queue_;
	USART_TypeDef* USARTx_;
	Dma dma_tx_;
	Dma dma_rx_;
	volatile size_t outputNDT_ = 0;
};

#endif /* _USART_H_ */
