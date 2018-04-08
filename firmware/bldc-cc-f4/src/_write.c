//
// This file is part of the µOS++ III distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// Do not include on semihosting and when freestanding
#if !defined(OS_USE_SEMIHOSTING) && !(__STDC_HOSTED__ == 0)

// ----------------------------------------------------------------------------

#include <errno.h>
#include "diag/Trace.h"
#include "stm32f4xx.h"
#include "bldc-include.h"

// ----------------------------------------------------------------------------

// When using retargetted configurations, the standard write() system call,
// after a long way inside newlib, finally calls this implementation function.

// Based on the file descriptor, it can send arrays of characters to
// different physical devices.

// Currently only the output and error file descriptors are tested,
// and the characters are forwarded to the trace device, mainly
// for demonstration purposes. Adjust it for your specific needs.

// For freestanding applications this file is not used and can be safely
// ignored.

ssize_t _write (int fd, const char* buf, size_t nbyte);


static uint32_t dma2_stream7_transmitting = 0;

void DMA2_Stream7_TC_callback(void)
{
	dma2_stream7_transmitting = 0;
}


ssize_t usart_write(USART_TypeDef *usart, const char* buf __attribute__((unused)), size_t nbyte __attribute__((unused)))
{
	size_t i = 0;

	for (i = 0; i < nbyte; i++) {
		while(!LL_USART_IsActiveFlag_TXE(usart))
			;
		LL_USART_TransmitData8(usart, buf[i]);
	}
}

ssize_t usart_write_dma(USART_TypeDef *usart, DMA_TypeDef *dma, uint32_t stream, const char* buf __attribute__((unused)), size_t nbyte __attribute__((unused)))
{
	dma2_stream7_transmitting = 1;
	LL_DMA_EnableIT_TC(dma, stream);
	LL_DMA_ConfigAddresses(dma, stream, (uint32_t)buf, LL_USART_DMA_GetRegAddr(usart), LL_DMA_GetDataTransferDirection(dma, stream));
	LL_DMA_SetDataLength(dma, stream, nbyte);
	LL_USART_EnableDMAReq_TX(usart);
	__disable_irq();
	LL_DMA_EnableStream(dma, stream);
	__enable_irq();
	while (dma2_stream7_transmitting);

	return nbyte;
}

#undef TRACE
ssize_t _write(int fd __attribute__((unused)), const char* buf __attribute__((unused)),
        size_t nbyte __attribute__((unused)))
{
	ssize_t ret = 0;

	if (fd == 1 || fd == 2) {
		ret = usart_write_dma(USART1, DMA2, LL_DMA_STREAM_7, buf, nbyte);
	}
#if defined(TRACE)
	// STDOUT and STDERR are routed to the trace device
	if (fd == 1 || fd == 2) {
		ret = trace_write(buf, nbyte);
	}
#endif // TRACE

	return ret;
}



#if ORIGINAL
ssize_t
_write (int fd, const char* buf, size_t nbyte);

ssize_t
_write (int fd __attribute__((unused)), const char* buf __attribute__((unused)),
	size_t nbyte __attribute__((unused)))
{
#if defined(TRACE)
  // STDOUT and STDERR are routed to the trace device
  if (fd == 1 || fd == 2)
    {
      return trace_write (buf, nbyte);
    }
#endif // TRACE

  errno = ENOSYS;
  return -1;
}
#endif
// ----------------------------------------------------------------------------

#endif // !defined(OS_USE_SEMIHOSTING) && !(__STDC_HOSTED__ == 0)
