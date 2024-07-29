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

#ifndef _CLHEAPALLOC_H_
#define _CLHEAPALLOC_H_


#if defined(__cplusplus)
extern "C" {
#endif


unsigned int cl_ha_gross_size(unsigned int uSize, unsigned int uBlocks);
int cl_ha_heap_init(void *pAddr, unsigned int uSize, unsigned int uBlocks);
void *cl_ha_alloc(void *pHeap, unsigned int uSize);
int cl_ha_free(void *pHeap, void *pAddr);
int cl_ha_available_size(void *pHeap);
int cl_ha_size(void *pHeap);
void cl_ha_debug(void *pHeap);

#if defined(__cplusplus)
}
#endif


#endif /* _CLHEAPALLOC_H_ */
