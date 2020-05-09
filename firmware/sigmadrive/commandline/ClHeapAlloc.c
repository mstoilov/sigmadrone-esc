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

#include "ClString.h"
#include "ClHeapAlloc.h"


#define HA_ALIGN(a, s) (((a) + (s) - 1) & ~((s) - 1))
#define HA_SIZE_ALIGN(a) HA_ALIGN(a, sizeof(unsigned int))
#define HA_ALIGNED_ADDRESS(a) (((a) & (sizeof(unsigned int) - 1)) == 0)
#define HA_HEAPMAP_EMPTY(m) ((m)->uSize == 0)


typedef struct _HEAPMAP
{
    unsigned int uAddr;
    unsigned int uSize;
} HEAPMAP;


typedef struct _HEAPHDR
{
    unsigned int uBaseAddr;
    unsigned int uSize;
    unsigned int uAvailSize;
    unsigned int uBlocks;
    unsigned int uAvailBlocks;
    HEAPMAP map[1];
} HEAPHDR;


int cl_ha_heap_init(void *pAddr, unsigned int uSize, unsigned int uBlocks)
{
    HEAPHDR *pHH = (HEAPHDR*) pAddr;

    if (uSize <= (sizeof(HEAPHDR) + sizeof(HEAPMAP) * uBlocks) || uBlocks < 1)
        return -1;

    cl_memset(pHH, 0, sizeof(*pHH));

    pHH->uSize = uSize - (sizeof(HEAPHDR) + sizeof(HEAPMAP) * uBlocks);
    pHH->uAvailSize = pHH->uSize;
    pHH->uBaseAddr = sizeof(HEAPHDR) + sizeof(HEAPMAP) * uBlocks;
    pHH->uBlocks = uBlocks;
    pHH->uAvailBlocks = uBlocks - 1;
    pHH->map[0].uAddr = pHH->uBaseAddr;
    pHH->map[0].uSize = pHH->uSize;
    pHH->map[1].uAddr = 0;
    pHH->map[1].uSize = 0;


    return 0;
}


static void cl_ha_set_block_size(void *pHeap, unsigned int uAddr, unsigned int uSize)
{
    unsigned int uCSize = (unsigned char) (uSize & 0xff) +
        (unsigned char) ((uSize >> 8) & 0xff) +
        (unsigned char) ((uSize >> 16) & 0xff);

    *(unsigned int *) ((char *) pHeap + uAddr) =
        ((uSize & 0x00ffffff) | ((~uCSize & 0xff) << 24));
}


static int cl_ha_get_block_size(unsigned int *puSize)
{
    unsigned int uSize = *puSize,
        uCSize = (unsigned char) (uSize & 0xff) +
        (unsigned char) ((uSize >> 8) & 0xff) +
        (unsigned char) ((uSize >> 16) & 0xff);

    *puSize = uSize & 0x00ffffff;

    return ((uSize >> 24) & 0xff) == (~uCSize & 0xff) ? 0: -1;
}


void *cl_ha_alloc(void *pHeap, unsigned int uSize)
{
    HEAPHDR *pHH = (HEAPHDR*) pHeap;
    HEAPMAP *pHM;
    unsigned int uAddr;
    void *pRet = (void*)0;

    if (pHH->uAvailBlocks == 0)
        return (void*)0;

    uSize = HA_SIZE_ALIGN(uSize + sizeof(unsigned int));
    for (pHM = pHH->map; !HA_HEAPMAP_EMPTY(pHM) && pHM->uSize < uSize; pHM++);

    if (pHM->uSize > uSize)
    {
        uAddr = pHM->uAddr;
        pHM->uSize -= uSize;
        pHM->uAddr += uSize;
        cl_ha_set_block_size(pHeap, uAddr, uSize);
        pHH->uAvailSize -= uSize;
        pRet = (void*)((char *) pHeap + uAddr + sizeof(unsigned int));
    }
    else if (pHM->uSize == uSize)
    {
        uAddr = pHM->uAddr;
        pHM->uSize = 0;
        pHM->uAddr = 0;
        cl_ha_set_block_size(pHeap, uAddr, uSize);
        do
        {
            pHM++;
            (pHM - 1)->uAddr = pHM->uAddr;
            (pHM - 1)->uSize = pHM->uSize;
        } while (pHM->uSize != 0);
        pHH->uAvailBlocks++;
        pHH->uAvailSize -= uSize;
        pRet = (void*)((char *) pHeap + uAddr + sizeof(unsigned int));
    }
    else
    {
        pRet = (void*)0;
    }

    return pRet;
}


int cl_ha_free(void *pHeap, void *pAddr)
{
    HEAPHDR *pHH = (HEAPHDR*) pHeap;
    HEAPMAP *pHM;
    unsigned int uBlkAddr, uBlkSize, uSize, uTmp;

    uBlkAddr = (unsigned int)((char*) pAddr - (char*)pHeap);
    uBlkAddr -= sizeof(unsigned int);

    if (!HA_ALIGNED_ADDRESS(uBlkAddr))
        return -1;

    uBlkSize = *((unsigned int*)((char*)pHeap + uBlkAddr));
    if (cl_ha_get_block_size(&uBlkSize) < 0 || uBlkSize == 0)
        return -1;
    *((unsigned int*)((char*)pHeap + uBlkAddr)) = 0;
    uSize = uBlkSize;

    for (pHM = pHH->map; pHM->uAddr <= uBlkAddr && !HA_HEAPMAP_EMPTY(pHM); pHM++);

    if (pHM > pHH->map && ((pHM - 1)->uAddr + (pHM - 1)->uSize) == uBlkAddr)
    {
        (pHM - 1)->uSize += uBlkSize;
        if ((pHM - 1)->uAddr + (pHM - 1)->uSize == pHM->uAddr)
        {
            (pHM - 1)->uSize += pHM->uSize;
            while (pHM->uSize)
            {
                pHM++;
                (pHM - 1)->uAddr = pHM->uAddr;
                (pHM - 1)->uSize = pHM->uSize;
            }
            pHH->uAvailBlocks++;
        }
    }
    else if (uBlkAddr + uBlkSize == pHM->uAddr && !HA_HEAPMAP_EMPTY(pHM))
    {
        pHM->uAddr -= uBlkSize;
        pHM->uSize += uBlkSize;
    }
    else
    {
        if (pHH->uAvailBlocks == 0)
            return -1;

        do
        {
            uTmp = pHM->uAddr;
            pHM->uAddr = uBlkAddr;
            uBlkAddr = uTmp;
            uTmp = pHM->uSize;
            pHM->uSize = uBlkSize;
            uBlkSize = uTmp;
            pHM++;
            if (pHM == (pHH->map + pHH->uBlocks))
                return -1;
        } while (uBlkSize != 0);
        pHH->uAvailBlocks--;
        pHM->uAddr = 0;
        pHM->uSize = 0;
    }

    pHH->uAvailSize += uSize;
    return 0;
}


int cl_ha_available_size(void *pHeap)
{
    HEAPHDR *pHH = (HEAPHDR*) pHeap;
    return pHH->uAvailSize;
}


int cl_ha_size(void *pHeap)
{
    HEAPHDR *pHH = (HEAPHDR*) pHeap;
    return pHH->uSize;
}


void cl_ha_debug(void *pHeap)
{
    HEAPHDR *pHH = (HEAPHDR*) pHeap;
    unsigned int i;

    cl_printf("Heap Addr: 0x%x\r\n", pHeap);
    cl_printf("uBaseAddr: %u\r\n", pHH->uBaseAddr);
    cl_printf("uSize: %u\r\n", pHH->uSize);
    cl_printf("uAvailSize: %u\r\n", pHH->uAvailSize);
    cl_printf("uBlocks: %u\r\n", pHH->uBlocks);
    cl_printf("uAvailBlocks: %u\r\n", pHH->uAvailBlocks);

    for (i = 0; i <= pHH->uBlocks; i++)
    {
        if (pHH->map[i].uSize)
            cl_printf("%u: uAddr: %u, uSize: %u\r\n", i, pHH->map[i].uAddr, pHH->map[i].uSize);
    }

    cl_printf("\r\n");
}
