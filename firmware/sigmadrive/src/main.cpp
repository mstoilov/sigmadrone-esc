#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "cmsis_device.h"

#include "interruptmanager.h"
#include "digitalin.h"
#include "digitalout.h"
#include "usart.h"

#include <iostream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

DigitalOut led_warn(PA_5, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullDown, DigitalOut::ActiveDefault, 0);
DigitalOut led_status(PA_6, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullNone, DigitalOut::ActiveLow, 0);
DigitalIn btn_user(PA_4, DigitalIn::PullDefault, DigitalIn::InterruptFalling);

extern USART* ptrUsart1;

int main(int argc, char* argv[])
{
	InterruptManager& im = InterruptManager::instance();

	uint32_t counter = 0;

	btn_user.Callback(&led_warn, &DigitalOut::Toggle);


	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

	// The standard output and the standard error should be forwarded to
	// the trace device. For this to work, a redirection in _write.c is
	// required.
	puts("Standard output message.");
	fprintf(stderr, "Standard error message.\n");

	// At this stage the system clock should have already been configured
	// at high speed.
	printf("System clock: %lu Hz\n", SystemCoreClock);

	char buffer[128];
	int ret = 0;
	while (1) {
		std::string tmp;
		HAL_Delay(10UL);
		led_status.Toggle();

//		printf("1234567890abcdefghijklmnopqrst1234567890abcdefghijklmnopqrst1234567890abcdefghijklmnopqrst1234567890abcdefghijklmnopqrst: %lu\n", counter++);
//		printf("1234567890abcdefghijklmnopqrst1234567890abcdefghijklmnopqrst1234567890abcdefghijklmnopqrst1234567890abcdefghijklmnopqrst: %lu\n", counter++);
//		printf("1234567890abcdefghijklmnopqrst1234567890abcdefghijklmnopqrst1234567890abcdefghijklmnopqrst1234567890abcdefghijklmnopqrst: %lu\n", counter++);
//		continue;

		uint32_t rxne = 0;//LL_USART_IsActiveFlag_RXNE(ptrUsart1->USARTx_);
		uint32_t ore = 0;//LL_USART_IsActiveFlag_ORE(ptrUsart1->USARTx_);
		printf("dma_rx.Enabled(): %s, SR: 0x%lx, RNE: %lu, ORE: %lu, counter: %d\n",
				ptrUsart1->IsEnable() ? "true" : "false",
				ptrUsart1->USARTx_->SR,
				ore,
				rxne,
				counter++);

		memset(buffer, 0, sizeof(buffer));
		while ((ret = read(1, buffer, sizeof(buffer) - 1)) > 0) {
			tmp += buffer;
			memset(buffer, 0, sizeof(buffer));
		}
		if (tmp.size()) {
			std::cout << tmp;
			memset(buffer, 0, sizeof(buffer));

		}

	}
}


extern "C" void SysTick_Handler(void)
{
#if defined(USE_HAL_DRIVER)
	HAL_IncTick();
#endif
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
