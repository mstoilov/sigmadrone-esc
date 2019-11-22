#include <errno.h>
#include <sys/types.h>
#include <string.h>
#include "uart.h"
#include "cdc_iface.h"

/*
 * Set up the remote terminal like this:
 * # stty -F /dev/ttyUSB0 115200 -echo igncr
 */

/*
 * uart1 is defined in appmain.cpp file.
 */
extern Uart uart1;
extern CdcIface usb_cdc;

#ifdef __cplusplus
extern "C"
#endif
int _write(int fd,
		const char* buf,
		size_t nbyte)
{
	static const char cr = '\r';
	size_t ret = 0;

	// STDOUT and STDERR are routed to the trace device
	if (fd == 1) {
		int* last_char = &_impure_ptr->_unspecified_locale_info;
		for (ret = 0; ret < nbyte; ret++) {
			if (buf[ret] != '\n' || *last_char == cr) {
				uart1.Transmit(&buf[ret], 1);
			} else {
				uart1.Transmit(&cr, 1);
				uart1.Transmit(&buf[ret], 1);
				*last_char = buf[ret];
			}
		}
		return ret;
	} else if (fd == 2) {
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
			;
		return ret;
	}

	errno = ENOSYS;
	return -1;
}
