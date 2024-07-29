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

#ifndef _CLDLIST_H_
#define _CLDLIST_H_

struct DLIST_HEAD
{
    struct DLIST_HEAD *pPrev;
    struct DLIST_HEAD *pNext;
};

#define DLIST_INIT(pHead) do { \
    (pHead)->pPrev = (pHead); (pHead)->pNext = (pHead); \
} while (0)

#define cl_dlist_parent_entry(ptr, type, member) \
    ((type *)((char *)(ptr) - (unsigned long)(&((type *)0)->member)))

#define cl_dlist_for_each(head, pos) \
    for (pos = (head)->pNext; pos != (head); pos = pos->pNext)

#ifdef __cplusplus
extern "C" {
#endif

void cl_dlist_add(struct DLIST_HEAD *pHead, struct DLIST_HEAD *pNew);
void cl_dlist_add_tail(struct DLIST_HEAD *pHead, struct DLIST_HEAD *pNew);
void cl_dlist_remove(struct DLIST_HEAD *pEntry);
void cl_dlist_replace(struct DLIST_HEAD *pOld, struct DLIST_HEAD *pNew);
int cl_dlist_is_empty(struct DLIST_HEAD *pHead);

#ifdef __cplusplus
}
#endif

#endif
