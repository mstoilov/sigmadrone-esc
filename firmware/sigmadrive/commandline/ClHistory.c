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
#include "ClPort.h"
#include "ClString.h"
#include "ClHistory.h"


#define HISTORY_MAGIC 0xabffcd19

typedef struct tagHISTNODE
{
	struct DLIST_HEAD list;
	unsigned int uSize;
	char szText[1];
} HISTNODE;


typedef struct tagSL_HISTORY
{
	unsigned int uMagic;
	struct DLIST_HEAD head;
	struct DLIST_HEAD *pCNode;
	unsigned int uSize;
	unsigned int uEntries;
} SL_HISTORY;


static SL_HISTORY hist;


int cl_history_init()
{
	SL_HISTORY *pHist = &hist;

	pHist->uMagic = HISTORY_MAGIC;
	DLIST_INIT(&pHist->head);
	pHist->pCNode = &pHist->head;
	pHist->uSize = 0;
	pHist->uEntries = 0;

	return 0;
}


int cl_history_is_empty()
{
	SL_HISTORY *pHist = &hist;

	return cl_dlist_is_empty(&pHist->head) ? 1 : 0;
}


void cl_history_head()
{
	SL_HISTORY *pHist = &hist;

	pHist->pCNode = &pHist->head;
}


const char *cl_history_next()
{
	SL_HISTORY *pHist = &hist;

	if (cl_dlist_is_empty(&pHist->head) || pHist->pCNode->pNext == &pHist->head)
		return (void*)0;

	pHist->pCNode = pHist->pCNode->pNext;

	return ((HISTNODE*)pHist->pCNode)->szText;
}


const char *cl_history_prev()
{
	SL_HISTORY *pHist = &hist;

	if (cl_dlist_is_empty(&pHist->head) || 
		pHist->pCNode->pPrev == &pHist->head ||
		pHist->pCNode == &pHist->head)
		return (void*)0;

	pHist->pCNode = pHist->pCNode->pPrev;
	if (pHist->pCNode == &pHist->head)
		return (void*)0;

	return ((HISTNODE*)pHist->pCNode)->szText;
}

static int cl_history_remove_node(struct DLIST_HEAD *pNode)
{
	SL_HISTORY *pHist = &hist;

	if (pNode == pHist->pCNode)
		pHist->pCNode = pHist->pCNode->pNext;
	pHist->uSize -= ((HISTNODE*)pNode)->uSize;
	pHist->uEntries -= 1;
	cl_dlist_remove(pNode);
	cl_mem_free((void*)pNode);

	return 0;
}

int cl_history_remove_tail()
{
	SL_HISTORY *pHist = &hist;

	if (cl_dlist_is_empty(&pHist->head))
		return -1;
	return cl_history_remove_node(pHist->head.pPrev);
}


int cl_history_remove_head()
{
	SL_HISTORY *pHist = &hist;

	if (cl_dlist_is_empty(&pHist->head))
		return -1;
	return cl_history_remove_node(pHist->head.pNext);
}

void cl_history_remove_all()
{
	while(cl_history_remove_tail() == 0)
		;
}

int cl_history_add(const char *pszText)
{
	SL_HISTORY *pHist = &hist;
	int iStrSize = cl_strlen(pszText) + 1;
	unsigned int uSize = sizeof(HISTNODE) + iStrSize;
	HISTNODE *pNode = (void*)0;

	if ((pNode = cl_mem_alloc(uSize)) == (void*)0)
		return -1;
	pHist->uSize += uSize;
	pHist->uEntries += 1;
	pNode->uSize = uSize;
	cl_strcpy(pNode->szText, pszText);
	cl_dlist_add(&pHist->head, &pNode->list);
	for (pNode = (HISTNODE *)pNode->list.pNext; ((struct DLIST_HEAD *)pNode) != &pHist->head; pNode = (HISTNODE *)pNode->list.pNext) {
		if (cl_strcmp(pNode->szText, pszText) == 0) {
			cl_history_remove_node(&pNode->list);
			break;
		}
	}
	return 0;
}


int cl_history_size()
{
	SL_HISTORY *pHist = &hist;

	return pHist->uSize;
}

int cl_history_entries()
{
	SL_HISTORY *pHist = &hist;

	return pHist->uEntries;
}
