/*
 *  Command Line (CL) Library
 *  Copyright (c) 2005-2019 Martin Stoilov
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Martin Stoilov <martin@sigmadrone.org>
 */

#include <stdlib.h>
#include <unistd.h>

#include "ClHeapAlloc.h"
#include "ClPort.h"

#ifndef CL_HAS_MALLOC

static void *pHeap = (void*)0;
int cl_mem_init(void *pAddr, unsigned int uSize, unsigned int uBlocks)
{
	pHeap = pAddr;
	if (cl_ha_heap_init(pHeap, uSize, uBlocks) < 0)
		return -1;
	return 0;
}
#endif

void *cl_mem_alloc(unsigned int uSize)
{
#ifdef CL_HAS_MALLOC
	return malloc(uSize);
#else
	return cl_ha_alloc(pHeap, uSize);
#endif
}


void cl_mem_free(void *pAddr)
{
#ifdef CL_HAS_MALLOC
	free(pAddr);
#else
	cl_ha_free(pHeap, pAddr);
#endif
}

int cl_putchar(int c)
{
	char ch = (char)c;
	write(1, &ch, sizeof(ch));
	return c;
}

int cl_getchar()
{
	char c = 0;
	read(0, &c, sizeof(c));
	return c;
}

