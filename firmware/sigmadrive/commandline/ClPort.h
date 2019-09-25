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

#ifndef _CLPORT_H_
#define _CLPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

int cl_putchar(int c);
int cl_getchar();

/*
 * Don't need to call this function if CL_HAS_MALLOC is defined,
 * because the cl_mem_alloc/cl_mem_free will use the malloc/free.
 */
#ifndef CL_HAS_MALLOC
int cl_mem_init(void *pAddr, unsigned int uSize, unsigned int uBlocks);
#endif

void *cl_mem_alloc(unsigned int uSize);
void cl_mem_free(void *pAddr);

#ifdef __cplusplus
}
#endif

#endif // _CLPORT_H_
