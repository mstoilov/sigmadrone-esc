#include <stdlib.h>


// Forward declaration

void
_exit(int code);


void
__attribute__((weak))
_exit(int code __attribute__((unused)))
{
	while (1)
		;
}

// ----------------------------------------------------------------------------

void
__attribute__((weak,noreturn))
abort(void)
{
	_exit(1);
}

// ----------------------------------------------------------------------------
