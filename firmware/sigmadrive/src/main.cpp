#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "cmsis_device.h"

#include "interruptmanager.h"
#include "digitalin.h"
#include "digitalout.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

DigitalOut led_warn(PA_5, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullDown, DigitalOut::ActiveDefault, 0);
DigitalOut led_status(PA_6, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullNone, DigitalOut::ActiveLow, 0);
DigitalIn btn_user(PA_4, DigitalIn::PullDefault, DigitalIn::InterruptFalling);


int main(int argc, char* argv[])
{
	InterruptManager& im = InterruptManager::instance();

	uint32_t counter = 0;

	btn_user.callback(&led_warn, &DigitalOut::toggle);


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

	while (1) {
		HAL_Delay(1000UL);
		led_status.toggle();

		printf("Counter: %lu\n", counter++);
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
