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

#include "ClDefs.h"
#include "ClHistory.h"
#include "ClPort.h"
#include "ClString.h"
#include "ClEditLine.h"

enum SPECIAL_KEYS {
	CTRL_A = 1,         /* Ctrl-a */
	CTRL_B = 2,         /* Ctrl-b */
	CTRL_C = 3,         /* Ctrl-c */
	CTRL_D = 4,         /* Ctrl-d */
	CTRL_E = 5,         /* Ctrl-e */
	CTRL_F = 6,         /* Ctrl-f */
	CTRL_H = 8,         /* Ctrl-h */
	TAB = 9,            /* Tab */
	CTRL_K = 11,        /* Ctrl-k */
	CTRL_L = 12,        /* Ctrl-l */
	ENTER = 13,         /* Enter */
	CTRL_N = 14,        /* Ctrl-n */
	CTRL_P = 16,        /* Ctrl-p */
	CTRL_T = 20,        /* Ctrl-t */
	CTRL_U = 21,        /* Ctrl-u */
	CTRL_W = 23,        /* Ctrl-w */
	ESC = 27,           /* Escape */
	BACKSPACE =  127    /* Backspace */
};


typedef struct tagSLEDITLINE
{
	char *pszBuffer;
	unsigned int uBufferSize;
	unsigned int uTextSize;
	unsigned int uBufPos;
	unsigned int uColPos;
	unsigned int uTermWidth;
	unsigned int uHistLevel;
} SLEDITLINE;


static int cl_editline_pos_left(SLEDITLINE *pEL);
static int cl_editline_pos_right(SLEDITLINE *pEL);
static int cl_editline_insert_char(SLEDITLINE *pEL, int iChar);
static int cl_editline_delete_char(SLEDITLINE *pEL);
static void cl_editline_home_key(SLEDITLINE *pEL);
static void cl_editline_end_key(SLEDITLINE *pEL);
static void cl_editline_left_key(SLEDITLINE *pEL);
static void cl_editline_right_key(SLEDITLINE *pEL);
static void cl_editline_up_key(SLEDITLINE *pEL);
static void cl_editline_down_key(SLEDITLINE *pEL);
static void cl_editline_invalidate_to_end(SLEDITLINE *pEL);
static void cl_term_cursor_left(int iOffset);
static void cl_term_cursor_right(int iOffset);
static void cl_term_cursor_up(int iOffset);
static void cl_term_cursor_down(int iOffset);
static int cl_term_output(SLEDITLINE *pEL, int iChar);
static int cl_term_cursor_col();
static int cl_term_width();


static int cl_term_width()
{
	int iWidth = 0;

	cl_printf("\0337");		 // Save cursor position
	cl_printf("\033[?25l");	 // Hide cursor
	cl_term_cursor_right(999);
	if ((iWidth = cl_term_cursor_col()) == 0)
		iWidth = 80;
	cl_printf("\0338");		 // Restore cursor position
	cl_printf("\033[?25h");	 // Show cursor

	return iWidth;
}


static void cl_term_cursor_left(int iOffset)
{
	if (iOffset > 0)
		cl_printf("\033[%dD", iOffset);
}


static void cl_term_cursor_right(int iOffset)
{
	if (iOffset > 0)
		cl_printf("\033[%dC", iOffset);
}


static void cl_term_cursor_up(int iOffset)
{
	if (iOffset > 0)
		cl_printf("\033[%dA", iOffset);
}


static void cl_term_cursor_down(int iOffset)
{
	if (iOffset > 0)
		cl_printf("\033[%dB", iOffset);
}


static void cl_term_cursor_nextline()
{
	cl_printf("\033D");
	cl_printf("\033[A");
	cl_printf("\033E");
}


static int cl_editline_pos_left(SLEDITLINE *pEL)
{
	if (pEL->uBufPos > 0)
	{
		pEL->uBufPos--;
		return 0;
	}

	return -1;
}


static int cl_editline_pos_right(SLEDITLINE *pEL)
{
	if (pEL->uBufPos < pEL->uTextSize)
	{
		pEL->uBufPos++;
		return 0;
	}

	return -1;
}


static int cl_term_cursor_col()
{
	int c;
	int iLine = 0;
	int iCol = 0;
	int repeat = 5;

	cl_printf("\033[6n");
	while (((c = cl_getchar()) != 27) && repeat)
		--repeat;
	if (c != 27)
		return 0;
	if (cl_getchar() == '[')
	{
		for(;;)
		{
			c = cl_getchar();
			if (c >= '0'&& c <= '9')
				iLine = 10*iLine + c - '0';
			else if (c == ';')
				break;
			else
				return 0;
		}

		for(;;)
		{
			c = cl_getchar();
			if (c >= '0' && c <= '9')
				iCol = 10*iCol + c - '0';
			else if (c == 'R')
				break;
			else
				return 0;
		}
	}

	return iCol;
}


static void cl_editline_invalidate_to_end(SLEDITLINE *pEL)
{
	int i;
	unsigned int uLineOff, uColPos;

	cl_printf("\033[?25l");	 // Hide cursor
	cl_printf("\033[0J");	 // Delete to the end of screen

	uColPos = pEL->uColPos;
	for (uLineOff = 0,i = pEL->uBufPos; i < pEL->uTextSize; i++)
		uLineOff += cl_term_output(pEL, (pEL->pszBuffer[i]));

	cl_term_cursor_left(pEL->uColPos - uColPos);
	cl_term_cursor_right(uColPos - pEL->uColPos);
	cl_term_cursor_up(uLineOff);
	pEL->uColPos = uColPos;

	cl_printf("\033[?25h");	 // Show cursor
}


static int cl_editline_insert_char(SLEDITLINE *pEL, int iChar)
{
	unsigned int i;
	unsigned char c = ' ';

	if (pEL->uBufPos >= pEL->uBufferSize - 1)
		return -1;

	for (i = pEL->uBufPos; i < pEL->uBufferSize - 1; i++)
	{
		if (c == '\0')
			break;
		c ^= pEL->pszBuffer[i];
		pEL->pszBuffer[i] ^= c;
		c ^= pEL->pszBuffer[i];
	}

	pEL->pszBuffer[i] = '\0';

//	for (i = pEL->uBufferSize - 1; i > pEL->uBufPos; i--)
//		pEL->pszBuffer[i] = pEL->pszBuffer[i-1];

	pEL->pszBuffer[pEL->uBufPos++] = iChar;
	pEL->uTextSize++;
	cl_term_output(pEL, iChar);
	cl_editline_invalidate_to_end(pEL);

	return 0;
}


static int cl_editline_delete_char(SLEDITLINE *pEL)
{
	unsigned int i;

	if (pEL->uBufPos < pEL->uTextSize)
	{
		for (i = pEL->uBufPos; i < pEL->uTextSize; i++)
			pEL->pszBuffer[i] = pEL->pszBuffer[i+1];

		pEL->uTextSize--;

		return 0;
	}

	return -1;
}


static void cl_editline_home_key(SLEDITLINE *pEL)
{
	unsigned int uLineOff, uColPos;

	uColPos = pEL->uTermWidth - (pEL->uTermWidth - 
								 pEL->uColPos + pEL->uBufPos) % pEL->uTermWidth;
	uLineOff = (pEL->uTermWidth - pEL->uColPos + pEL->uBufPos) / pEL->uTermWidth;

	cl_printf("\033[?25l");
	cl_term_cursor_left(pEL->uColPos - uColPos);
	cl_term_cursor_right(uColPos - pEL->uColPos);
	cl_term_cursor_up(uLineOff);
	cl_printf("\033[?25h");

	pEL->uColPos = uColPos;
	pEL->uBufPos = 0;
}


static void cl_editline_end_key(SLEDITLINE *pEL)
{
	unsigned int uLineOff, uColPos;

	uColPos = ((int)(pEL->uColPos + pEL->uTextSize - pEL->uBufPos)) % pEL->uTermWidth;
	uLineOff = ((int)(pEL->uColPos + pEL->uTextSize - pEL->uBufPos)) / pEL->uTermWidth;

	cl_printf("\033[?25l");
	cl_term_cursor_left(pEL->uColPos - uColPos);
	cl_term_cursor_right(uColPos - pEL->uColPos);
	cl_term_cursor_down(uLineOff);
	cl_printf("\033[?25h");

	pEL->uColPos = uColPos;
	pEL->uBufPos = pEL->uTextSize;
}


static void cl_editline_left_key(SLEDITLINE *pEL)
{
	if (cl_editline_pos_left(pEL) >= 0)
	{
		if (--(pEL->uColPos) == 0)
		{
			pEL->uColPos = pEL->uTermWidth;
			cl_term_cursor_right(pEL->uTermWidth);
			cl_term_cursor_up(1);
		}
		else
		{
			cl_term_cursor_left(1);
		}
	}
}


static void cl_editline_right_key(SLEDITLINE *pEL)
{
	if (cl_editline_pos_right(pEL) >= 0)
	{
		if (++(pEL->uColPos) > pEL->uTermWidth)
		{
			pEL->uColPos = 1;
			cl_term_cursor_left(pEL->uTermWidth);
			cl_term_cursor_down(1);
		}
		else
		{
			cl_term_cursor_right(1);
		}
	}
}


static void cl_editline_up_key(SLEDITLINE *pEL)
{
	const char *pszText;

	if ((pszText = cl_history_next()) == (void*)0)
		return;

	if (pEL->uHistLevel == 0)
		cl_history_add(pEL->pszBuffer);

	cl_editline_home_key(pEL);
	cl_memset(pEL->pszBuffer, 0, pEL->uBufferSize);
	cl_strcpy(pEL->pszBuffer, pszText);
	pEL->uTextSize = cl_strlen(pEL->pszBuffer);
	pEL->uBufPos = 0;
	cl_editline_invalidate_to_end(pEL);
	cl_editline_end_key(pEL);
	pEL->uHistLevel += 1;
}


static void cl_editline_down_key(SLEDITLINE *pEL)
{
	const char *pszText;

	if ((pszText = cl_history_prev()) == (void*)0)
		return;

	cl_editline_home_key(pEL);
	cl_memset(pEL->pszBuffer, 0, pEL->uBufferSize);
	cl_strcpy(pEL->pszBuffer, pszText);
	pEL->uTextSize = cl_strlen(pEL->pszBuffer);
	pEL->uBufPos = 0;
	cl_editline_invalidate_to_end(pEL);
	cl_editline_end_key(pEL);
	if ((pEL->uHistLevel -= 1) == 0)
	{
		cl_history_remove_head();
		cl_history_head();
	}
}


static void cl_editline_del_key(SLEDITLINE *pEL)
{
	if (cl_editline_delete_char(pEL) >= 0)
		cl_editline_invalidate_to_end(pEL);
}


static void cl_editline_back_key(SLEDITLINE *pEL)
{
	if (cl_editline_pos_left(pEL) >= 0)
	{
		if (--(pEL->uColPos) == 0)
		{
			pEL->uColPos = pEL->uTermWidth;
			cl_term_cursor_right(pEL->uTermWidth);
			cl_term_cursor_up(1);
		}
		else
		{
			cl_term_cursor_left(1);
		}

		if (cl_editline_delete_char(pEL) >= 0)
			cl_editline_invalidate_to_end(pEL);
	}
}


static int cl_term_output(SLEDITLINE *pEL, int iChar)
{
	cl_putchar(iChar);

	if (++(pEL->uColPos) > pEL->uTermWidth)
	{
		cl_term_cursor_nextline();
		pEL->uColPos = 1;
		return 1;
	}

	return 0;
}


void cl_term_enable_wrap()
{
	cl_printf("\033[?7h");
}


void cl_term_disable_wrap()
{
	cl_printf("\033[?7l");
}

int cl_editline(const char *pszPrompt, char *pszBuffer, unsigned int uBufferSize, unsigned int uHistoryEntries)
{
	SLEDITLINE EL;
	int iChar;

	if (pszBuffer == NULL || uBufferSize == 0)
		return -1;

	cl_memset(pszBuffer, 0, uBufferSize);
	cl_memset(&EL, 0, sizeof(EL));
	EL.pszBuffer = pszBuffer;
	EL.uBufferSize = uBufferSize -1;
	if ((EL.uTermWidth = cl_term_width()) == 0)
		EL.uTermWidth = 80;


	while (cl_history_entries() > uHistoryEntries)
		cl_history_remove_tail();
	cl_history_head();

	if (pszPrompt)
		cl_printf("%d, %s", EL.uTermWidth, pszPrompt);

	EL.uColPos = cl_term_cursor_col();

	while ( (iChar = cl_getchar()) != '\n' && iChar != '\r')
	{
		switch (iChar)
		{
		case 3:
			return -1;
			break;

		case 27:
			if ((iChar = cl_getchar()) == 91)
			{
				iChar = cl_getchar();
				switch (iChar)
				{
				case 'A': /* Up */
					cl_editline_up_key(&EL);
					break;

				case 'B': /* Down */
					cl_editline_down_key(&EL);
					break;

				case 'C': /* Right */
					cl_editline_right_key(&EL);
					break;

				case 'D': /* Left */
					cl_editline_left_key(&EL);
					break;

				case 51: /* Del */
					cl_getchar();
					cl_editline_del_key(&EL);
					break;

				case 49: /* Home */
					cl_getchar();
					cl_editline_home_key(&EL);
					break;

				case 52: /* End */
					cl_getchar();
					cl_editline_end_key(&EL);
					break;

				case 50:
				case 53:
				case 54:
					cl_getchar();
					break;

				default:
					continue;
				};
			}
			else if (iChar == 'O')
			{
				iChar = cl_getchar();
				switch (iChar)
				{
				case 'H': /* Home */
					cl_editline_home_key(&EL);
					break;
				case 'F': /* End */
					cl_editline_end_key(&EL);
					break;
				}
			}
			break;
		case BACKSPACE:
		case CTRL_H: /* Backspace */
			cl_editline_back_key(&EL);
			break;
		case TAB:
			break;
		case CTRL_A: /* Home */
			cl_editline_home_key(&EL);
			break;
		case CTRL_E: /* End */
			cl_editline_end_key(&EL);
			break;
		case CTRL_D: /* Del */
			cl_editline_del_key(&EL);
			break;
		case CTRL_B: /* Left */
			cl_editline_left_key(&EL);
			break;
		case CTRL_F: /* Right */
			cl_editline_right_key(&EL);
			break;
		default:
			cl_editline_insert_char(&EL, iChar);
			break;

		};
	}

	cl_editline_end_key(&EL);

	if (EL.uHistLevel != 0)
	{
		cl_history_remove_head();
		cl_history_head();
	}

	if (EL.uTextSize)
		cl_history_add(pszBuffer);

	return EL.uTextSize;
}
