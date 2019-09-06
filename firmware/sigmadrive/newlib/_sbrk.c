#include <errno.h>
#include <sys/types.h>

caddr_t _sbrk (int incr)
{
   register char * stack_ptr asm ("sp");
   extern char end asm ("end");
   static char *heap_end;
   char *prev_heap_end;

   if (heap_end == NULL)
        heap_end = &end;

    prev_heap_end = heap_end;

    if(heap_end + incr > stack_ptr)
    {
        errno = ENOMEM;
        return (caddr_t) -1;
    }

    heap_end += incr;

   return (caddr_t) prev_heap_end;
}
