#include <errno.h>
#include <sys/types.h>
#include <string.h>
#include "application/uart.h"
#include "application/cdc_iface.h"
#include "cmsis_os2.h"

/*
 * Set up the remote terminal like this:
 * # stty -F /dev/ttyUSB0 115200 -echo igncr
 * # stty -F /dev/ttyACM0 115200 -echo igncr
 * EZSync: GND (black), TXD (orange), RXD (yellow)
 */

/*
 * uart1 is defined in appmain.cpp file.
 */
extern Uart uart1;
extern Uart uart8;
extern CdcIface usb_cdc;

#ifdef __cplusplus
extern "C"
#endif

int _write(int fd,
		const char* buf,
		size_t nbyte)
{
	// STDOUT and STDERR are routed to the trace device
	if (fd == 1) {
		return uart1.Transmit(buf, nbyte);
	} else if (fd == 2) {
		return uart8.Transmit(buf, nbyte);
	} else if (fd == 3) {
		return usb_cdc.Transmit(buf, nbyte);
	}

	errno = ENOSYS;
	return -1;
}


#ifdef __cplusplus
extern "C"
#endif
int _read(int fd __attribute__((unused)), char* buf __attribute__((unused)),
    int nbyte __attribute__((unused)))
{
	int ret = 0;

	// STDIN are routed to the trace device
	if (fd == 0) {
		while ((ret = uart1.Receive(buf, nbyte)) <= 0)
			osDelay(50);
		return ret;
	} else if (fd == 3) {
		return usb_cdc.Receive(buf, nbyte);
	}

	errno = ENOSYS;
	return -1;
}
