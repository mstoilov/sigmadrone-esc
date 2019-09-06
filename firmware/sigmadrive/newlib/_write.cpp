#include <errno.h>
#include <sys/types.h>
#include <string.h>
#include "uart.h"

/*
 * Set up the remote terminal like this:
 * # stty -F /dev/ttyUSB0 230400 -echo igncr
 */

/*
 * uart1 is defined in appmain.cpp file.
 */
extern Uart uart1;


#ifdef __cplusplus
extern "C"
#endif
ssize_t _write(int fd __attribute__((unused)),
		const char* buf __attribute__((unused)),
		size_t nbyte __attribute__((unused)))
{
	// STDOUT and STDERR are routed to the trace device
	if (fd == 1 || fd == 2) {
		return uart1.transmit(buf, nbyte);
	}
	errno = ENOSYS;
	return -1;
}
