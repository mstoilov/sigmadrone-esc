#include <errno.h>
#include <sys/types.h>

caddr_t _sbrk_old(int incr)
{
	extern char heapstart asm ("heapstart");
	extern char heapend asm ("heapend");
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == NULL)
		heap_end = &heapstart;

	prev_heap_end = heap_end;

	incr = (incr + 3) & (~3); // align value to 4

	if (heap_end + incr > &heapend) {
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;
	return (caddr_t) prev_heap_end;
}

caddr_t _sbrk(int incr)
{
	extern char end asm ("end");
	static char *heap_end;
	char *prev_heap_end;
	extern char _Heap_Limit;

	if (heap_end == NULL)
		heap_end = &end;

	prev_heap_end = heap_end;

	incr = (incr + 3) & (~3); // align value to 4

	if (heap_end + incr > &_Heap_Limit) {
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;
	return (caddr_t) prev_heap_end;
}