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

#include "ClDList.h"


static void __dlist_add(struct DLIST_HEAD *pNew, 
                        struct DLIST_HEAD *pPrev,
                        struct DLIST_HEAD *pNext)
{
    pNew->pNext = pNext;
    pNew->pPrev = pPrev;
    pPrev->pNext = pNew;
    pNext->pPrev = pNew;
}


static void __dlist_del_middle(struct DLIST_HEAD *pPrev, struct DLIST_HEAD *pNext)
{
    pPrev->pNext = pNext;
    pNext->pPrev = pPrev;
}


void cl_dlist_add(struct DLIST_HEAD *pHead, struct DLIST_HEAD *pNew)
{
    __dlist_add(pNew, pHead, pHead->pNext);
}


void cl_dlist_add_tail(struct DLIST_HEAD *pHead, struct DLIST_HEAD *pNew)
{
    __dlist_add(pNew, pHead->pPrev, pHead);
}


void cl_dlist_remove(struct DLIST_HEAD *pEntry)
{
    __dlist_del_middle(pEntry->pPrev, pEntry->pNext);
    pEntry->pPrev = (void*) 0;
    pEntry->pNext = (void*) 0;
}


void cl_dlist_replace(struct DLIST_HEAD *pOld, struct DLIST_HEAD *pNew)
{
    DLIST_INIT(pNew);

    if (pOld->pNext != pOld && pOld->pPrev != pOld)
    {
        pNew->pNext = pOld->pNext;
        pOld->pNext->pPrev = pNew;
        pNew->pPrev = pOld->pPrev;
        pOld->pPrev->pNext = pNew;
    }

    pOld->pPrev = (void*) 0;
    pOld->pNext = (void*) 0;
}


int cl_dlist_is_empty(struct DLIST_HEAD *pHead)
{
    return (pHead->pNext == pHead);
}

