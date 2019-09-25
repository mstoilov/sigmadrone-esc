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

#ifndef _CLSTRING_H_
#define _CLSTRING_H_

#include <stdarg.h>


#define __MIN(a, b) ((a) < (b) ? (a): (b))
#define __MAX(a, b) ((a) > (b) ? (a): (b))
#define CPY2SZ(d, s, n) do { cl_memcpy(d, s, n); ((char *) (d))[n] = '\0'; } while (0)
#define STRNCPY(d, s, n) cl_strncpy(d, s, n)[(n) - 1] = '\0'
#define SSTRNCPY(d, s) STRNCPY(d, s, sizeof(d))
#define CSTRSIZE(s) (sizeof(s) - 1)
#define SKIPCHARS(p, c) for (; *(p) && cl_strchr(c, *(p)); (p)++)
#define SKIPSPACE(p) SKIPCHARS(p, " \t")
#define STRINGEMPTY(s) (*(s) == '\0')
#define SETEMPTYSTRING(s) (s)[0] = '\0'
#define LOCHAR(c) ((c) >= 'A' && (c) <= 'Z' ? 'a' + ((c) - 'A'): (c))
#define EOL_SEQUENCE "\n"

#ifdef __cplusplus
extern "C" {
#endif

void *cl_memcpy(void *pDest, void const *pSrc, unsigned int uSize);
void *cl_memset(void *pData, unsigned int uByte, unsigned int uSize);
int cl_memcmp(void const *pPtrA, void const *pPtrB, unsigned int uSize);
void *cl_memchr(void const *pData, unsigned int uByte, unsigned int uSize);
char *cl_strchr(char const *pStr, int iCh);
char *cl_strrchr(char const *pStr, int iCh);
char *cl_strcpy(char *pDest, char const *pSrc);
char *cl_strncpy(char *pDest, const char *pSrc, unsigned int uSize);
char *cl_strcat(char *pDest, char const *pSrc);
int cl_strcmp(char const *pStr1, char const *pStr2);
int cl_strncmp(char const *pStr1, char const *pStr2, unsigned int uSize);
int cl_strlen(char const *pStr);
char *cl_strltrim(char *pStr, char const *pTChrs);
char *cl_strrtrim(char *pStr, char const *pTChrs);
char *cl_strtrim(char *pStr, char const *pTChrs);
char *cl_strstr(char const *pStr, char const *pSStr);
char *cl_stristr(char const *pStr, char const *pSStr);
char *cl_strlwr(char *pStr);
char *cl_strupr(char *pStr);
char *cl_ulongtostr(unsigned long ulVal, char *pStr, int iSize);
char *cl_longtostr(long lVal, char *pStr, int iSize);
char *cl_uinttohstr(unsigned int uVal, char *pStr, int iSize);
char *cl_ulongtohstr(unsigned long ulVal, char *pStr, int iSize);
int cl_vformat(int (*pfOut)(char const *, int, void *), void *pPrivate, 
			   char const *pszFmt, va_list Args);
int cl_vsprintf(char *pszBuffer, int iSize, char const *pszFmt, va_list Args);
int cl_sprintf(char *pszBuffer, int iSize, char const *pszFmt, ...);
int cl_printf(char const *pszFmt, ...);
long cl_atol(char const *pszStr);
unsigned long cl_ahtoulong(const char* pszNum);
unsigned long cl_atoulong(const char* pszNum);
void cl_printhex(unsigned char uNum);

#ifdef __cplusplus
}
#endif

#endif
