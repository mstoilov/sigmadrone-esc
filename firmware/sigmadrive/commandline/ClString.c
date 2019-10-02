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
#include "ClPort.h"
#include "ClString.h"


struct StrOutData
{
	char *pszBuffer;
	int iSize;
	int iCurr;
};


void *cl_memcpy(void *pDest, void const *pSrc, unsigned int uSize)
{
	unsigned char *pDD, *pSD;

	if (pDest <= pSrc)
	{
		for (pDD = (unsigned char *) pDest, pSD = (unsigned char *) pSrc; uSize > 0; uSize--)
			*pDD++ = *pSD++;
	}
	else
	{
		for (pDD = ((unsigned char *) pDest + uSize - 1), pSD = ((unsigned char *) pSrc + uSize - 1);
			 uSize > 0; uSize--)
			*pDD-- = *pSD--;
	}

	return pDest;
}


void *cl_memset(void *pData, unsigned int uByte, unsigned int uSize)
{
	unsigned char *pTmp;

	for (pTmp = (unsigned char *) pData; uSize > 0; uSize--)
		*pTmp++ = (unsigned char) uByte;

	return pData;
}


int cl_memcmp(void const *pPtrA, void const *pPtrB, unsigned int uSize)
{
	unsigned char const *pA = (unsigned char const *) pPtrA, *pB = (unsigned char const *) pPtrB;

	for (; uSize > 0; uSize--, pA++, pB++)
	{
		if (*pA != *pB)
			return (int) *pA - (int) *pB;
	}

	return 0;
}


void *cl_memchr(void const *pData, unsigned int uByte, unsigned int uSize)
{
	unsigned char *pTmp = (unsigned char *) pData;

	for (; uSize > 0; uSize--, pTmp++)
		if (*pTmp == uByte)
			return pTmp;

	return NULL;
}


char *cl_strchr(char const *pStr, int iCh)
{
	for (; *pStr; pStr++)
		if (*pStr == iCh)
			return (char *) pStr;

	return NULL;
}


char *cl_strrchr(char const *pStr, int iCh)
{
	char *pTmp = NULL;
	for (; *pStr; pStr++)
		if (*pStr == iCh)
			pTmp = (char *) pStr;
	return pTmp;
}


char *cl_strcpy(char *pDest, char const *pSrc)
{
	char *pTmp = pDest;
	while (*pSrc)
		*pTmp++ = *pSrc++;
	*pTmp = 0;
	return pDest;
}


char *cl_strncpy(char *pDest, const char *pSrc, unsigned int uSize)
{
	char *s = pDest;

	while (uSize) {
		if ((*s = *pSrc) != 0) pSrc++; /* Need to fill tail with 0s. */
		++s;
		--uSize;
	}
	
	return pDest;
}


char *cl_strcat(char *pDest, char const *pSrc)
{
	int slen = cl_strlen(pDest);
	return cl_strcpy(pDest + slen, pSrc);
}


int cl_strcmp(char const *pStr1, char const *pStr2)
{
	for (; *pStr1 && *pStr2; pStr1++, pStr2++)
		if (*pStr1 != *pStr2)
			break;
	return (int) *pStr1 - (int) *pStr2;
}


int cl_strncmp(char const *pStr1, char const *pStr2, unsigned int uSize)
{
	for (; uSize && *pStr1 && *pStr2; pStr1++, pStr2++, uSize--)
		if (*pStr1 != *pStr2)
			break;
	return uSize ? (int) *pStr1 - (int) *pStr2: 0;
}


int cl_strlen(char const *pStr)
{
	char const *pTmp = pStr;
	for (; *pTmp; pTmp++);
	return (int) (pTmp - pStr);
}


char *cl_strltrim(char *pStr, char const *pTChrs)
{
	int ii, jj;

	for (ii = 0; pStr[ii] != '\0' && cl_strchr(pTChrs, pStr[ii]) != NULL; ii++);
	if (ii > 0 && pStr[ii] != '\0')
	{
		for (jj = ii; pStr[jj] != '\0'; jj++)
			pStr[jj - ii] = pStr[jj];
		pStr[jj - ii] = pStr[jj];
	}
	return pStr;
}


char *cl_strrtrim(char *pStr, char const *pTChrs)
{
	int ii;
	for (ii = cl_strlen(pStr) - 1; ii >= 0 && cl_strchr(pTChrs, pStr[ii]) != NULL; ii--)
		pStr[ii] = '\0';
	return pStr;
}


char *cl_strtrim(char *pStr, char const *pTChrs)
{
	return cl_strrtrim(cl_strltrim(pStr, pTChrs), pTChrs);
}


char *cl_strstr(char const *pStr, char const *pSStr)
{
	int sslen = cl_strlen(pSStr), mpos = 0, mcc = *pSStr;
	if (!sslen)
		return (char *) pStr;
	for (; *pStr; pStr++)
		if (*pStr == mcc)
		{
			if (++mpos == sslen)
				return (char *) pStr - sslen + 1;
			mcc = pSStr[mpos];
		}
		else if (mpos)
			mpos = 0, mcc = *pSStr;
	return NULL;
}


char *cl_stristr(char const *pStr, char const *pSStr)
{
	int sslen = cl_strlen(pSStr), mpos = 0, mcc = SLLOCHAR(*pSStr);
	if (!sslen)
		return (char *) pStr;
	for (; *pStr; pStr++)
		if (SLLOCHAR(*pStr) == mcc)
		{
			if (++mpos == sslen)
				return (char *) pStr - sslen + 1;
			mcc = SLLOCHAR(pSStr[mpos]);
		}
		else if (mpos)
			mpos = 0, mcc = SLLOCHAR(*pSStr);
	return NULL;
}


char *cl_strlwr(char *pStr)
{
	char *pTmp = pStr;
	for (; *pTmp; pTmp++)
		*pTmp = (char) SLLOCHAR(*pTmp);
	return pStr;
}


char *cl_strupr(char *pStr)
{
	char *pTmp = pStr;
	for (; *pTmp; pTmp++)
		*pTmp = (char) SLHICHAR(*pTmp);
	return pStr;
}


char *cl_ulongtostr(unsigned long ulVal, char *pStr, int iSize)
{
	char szBuffer[32];
	char *pTmp = szBuffer + sizeof(szBuffer) - 1;

	for (*pTmp = '\0'; ulVal && pTmp > szBuffer; ulVal /= 10)
		*--pTmp = "0123456789"[ulVal % 10];
	return cl_strcpy(pStr, pTmp == (szBuffer + sizeof(szBuffer) - 1) ? "0": pTmp);
}


char *cl_longtostr(long lVal, char *pStr, int iSize)
{

	if (lVal >= 0)
		return cl_ulongtostr((unsigned long) lVal, pStr, iSize);
	*pStr = '-';
	return cl_ulongtostr((unsigned long) -lVal, pStr + 1, iSize - 1) - 1;
}


char *cl_uinttohstr(unsigned int uVal, char *pStr, int iSize)
{
	char szBuffer[1 + sizeof(unsigned int) * 2];
	char *pTmp = szBuffer + sizeof(szBuffer) - 1;

	cl_memset(szBuffer, (unsigned int) '0', sizeof(szBuffer));
	for (*pTmp = '\0'; uVal && pTmp > szBuffer; uVal >>= 4)
		*--pTmp = "0123456789abcdef"[uVal & 0x0f];
	return cl_strcpy(pStr, szBuffer);
}


char *cl_ulongtohstr(unsigned long ulVal, char *pStr, int iSize)
{
	char szBuffer[1 + sizeof(unsigned long) * 2];
	char *pTmp = szBuffer + sizeof(szBuffer) - 1;

	cl_memset(szBuffer, (unsigned int) '0', sizeof(szBuffer));
	for (*pTmp = '\0'; ulVal && pTmp > szBuffer; ulVal >>= 4)
		*--pTmp = "0123456789abcdef"[ulVal & 0x0f];
	return cl_strcpy(pStr, szBuffer);
}

int cl_vformat(int (*pfOut)(char const *, int, void *), void *pPrivate, char const *pszFmt, va_list Args)
{
	int iStart = 0, iRes, i;
	unsigned int uArg;
	long lArg;
	unsigned long ulArg;
	char const *pszArg;
	char szBuffer[32];
																													 
	for (i = 0; pszFmt[i]; i++)
	{
		if (pszFmt[i] != '%')
			continue;
		switch (pszFmt[i + 1])
		{
		case '%':
			if ((iRes = (*pfOut)(pszFmt + iStart, i + 1 - iStart, pPrivate)) < 0)
				return iRes;
			iStart = ++i + 1;
			break;
		case 's':
			pszArg = va_arg(Args, char const *);
			if ((iRes = (*pfOut)(pszFmt + iStart, i - iStart, pPrivate)) < 0 ||
				(iRes = (*pfOut)(pszArg, cl_strlen(pszArg), pPrivate)) < 0)
				return iRes;
			iStart = ++i + 1;
			break;
		case 'l':
			switch (pszFmt[i + 2])
			{
			case 'i':
			case 'd':
				lArg = va_arg(Args, long);
				cl_longtostr(lArg, szBuffer, sizeof(szBuffer) - 1);
				if ((iRes = (*pfOut)(pszFmt + iStart, i - iStart, pPrivate)) < 0 ||
					(iRes = (*pfOut)(szBuffer, cl_strlen(szBuffer), pPrivate)) < 0)
					return iRes;
				iStart = (i += 2) + 1;
				break;
			case 'u':
				ulArg = va_arg(Args, unsigned long);
				cl_ulongtostr(ulArg, szBuffer, sizeof(szBuffer) - 1);
				if ((iRes = (*pfOut)(pszFmt + iStart, i - iStart, pPrivate)) < 0 ||
					(iRes = (*pfOut)(szBuffer, cl_strlen(szBuffer), pPrivate)) < 0)
					return iRes;
				iStart = (i += 2) + 1;
				break;
			case 'x':
				ulArg = va_arg(Args, unsigned long);
				cl_ulongtohstr(ulArg, szBuffer, sizeof(szBuffer) - 1);
				if ((iRes = (*pfOut)(pszFmt + iStart, i - iStart, pPrivate)) < 0 ||
					(iRes = (*pfOut)(szBuffer, cl_strlen(szBuffer), pPrivate)) < 0)
					return iRes;
				iStart = (i += 2) + 1;
				break;
			}
			break;
		case 'i':
		case 'd':
			lArg = (long) va_arg(Args, int);
			cl_longtostr(lArg, szBuffer, sizeof(szBuffer) - 1);
			if ((iRes = (*pfOut)(pszFmt + iStart, i - iStart, pPrivate)) < 0 ||
				(iRes = (*pfOut)(szBuffer, cl_strlen(szBuffer), pPrivate)) < 0)
				return iRes;
			iStart = ++i + 1;
			break;
		case 'u':
			ulArg = (unsigned long) va_arg(Args, unsigned int);
			cl_ulongtostr(ulArg, szBuffer, sizeof(szBuffer) - 1);
			if ((iRes = (*pfOut)(pszFmt + iStart, i - iStart, pPrivate)) < 0 ||
				(iRes = (*pfOut)(szBuffer, cl_strlen(szBuffer), pPrivate)) < 0)
				return iRes;
			iStart = ++i + 1;
			break;
		case 'p':
		case 'x':
			uArg = va_arg(Args, unsigned int);
			cl_uinttohstr(uArg, szBuffer, sizeof(szBuffer) - 1);
			if ((iRes = (*pfOut)(pszFmt + iStart, i - iStart, pPrivate)) < 0 ||
				(iRes = (*pfOut)(szBuffer, cl_strlen(szBuffer), pPrivate)) < 0)
				return iRes;
			iStart = ++i + 1;
			break;
		}
	}
	if (iStart < i && (iRes = (*pfOut)(pszFmt + iStart, i - iStart, pPrivate)) < 0)
		return iRes;
	return 0;
}

static int cl_strout(char const *pszStr, int iSize, void *pPrivate)
{
	struct StrOutData *pSOD = (struct StrOutData *) pPrivate;
																													 
	if (iSize > 0)
	{
		if ((pSOD->iCurr + iSize) <= pSOD->iSize)
		{
			cl_memcpy(pSOD->pszBuffer + pSOD->iCurr, pszStr, (unsigned int) iSize);
			pSOD->pszBuffer[pSOD->iCurr + iSize] = '\0';
		}
		pSOD->iCurr += iSize;
	}
	return 0;
}

																													 
static int cl_stdout(char const *pszStr, int iSize, void *pPrivate)
{
	int iRet = 0;
	int iWritten = 0;
	unsigned int uOffset = 0;

	while (iSize)
	{
		iWritten = cl_write(pszStr + uOffset, iSize);
		if (iWritten < 0)
			return -1;
		iSize -= iWritten;
		uOffset += iWritten;
		iRet += iWritten;
	}

	return iRet;
}
																													 
int cl_vsprintf(char *pszBuffer, int iSize, char const *pszFmt, va_list Args)
{
	int iFmtRes;
	struct StrOutData SOD;
																													 
	SOD.pszBuffer = pszBuffer;
	SOD.iSize = iSize - 1;
	SOD.iCurr = 0;
																													 
	*pszBuffer = '\0';
	if ((iFmtRes = cl_vformat(cl_strout, &SOD, pszFmt, Args)) < 0)
		return iFmtRes;
																													 
	return SOD.iCurr;
}


int cl_sprintf(char *pszBuffer, int iSize, char const *pszFmt, ...)
{
	int iRes;
	va_list Args;
																													 
	va_start(Args, pszFmt);
																													 
	iRes = cl_vsprintf(pszBuffer, iSize, pszFmt, Args);
																													 
	va_end(Args);
	return iRes;
}

int cl_printf(char const *pszFmt, ...)
{
	int iFmtRes;
	va_list Args;
																													 
	va_start(Args, pszFmt);

	iFmtRes = cl_vformat(cl_stdout, NULL, pszFmt, Args);

	va_end(Args);
	return iFmtRes;
}
																												  

long cl_atol(char const *pszStr)
{
	char const *pszTmp;
	unsigned long ulValue = 0, ulMult = 1;
	long lSign = +1;

	if (*pszStr == '-')
		lSign = -1, pszStr++;
	else if (*pszStr == '+')
		pszStr++;
	for (pszTmp = pszStr; SLISDIGIT(*pszTmp); pszTmp++);
	for (--pszTmp; pszTmp >= pszStr; pszTmp--, ulMult *= 10)
		ulValue += ulMult * (unsigned long) (*pszTmp - '0');
	return lSign > 0 ? (long) ulValue: - (long) ulValue;
}

unsigned long cl_ahtoulong(const char* pszNum)
{
	char szTemp[17];
	const char *pszEnd;
	unsigned long uRet = 0;
	int i;
 
	for (i = 0; i < sizeof(szTemp); i++)
		szTemp[i] = '0';
	szTemp[sizeof(szTemp)-1] = '\0';
 
	for (pszEnd = pszNum; *pszEnd != '\0'; pszEnd++);
 
	pszEnd--;
	for (i = sizeof(szTemp)-2; i>=0 && pszEnd >= pszNum; i--, pszEnd--)
		szTemp[i] = (*pszEnd == 'x' || *pszEnd == 'X') ? '0' : *pszEnd;

	for (i = 0; i < sizeof(szTemp)-1; i++)
	{
		uRet <<= 4;
		if (szTemp[i] >= '0' && szTemp[i] <= '9')
			uRet |= (szTemp[i] - '0');
		else if (szTemp[i] >= 'a' && szTemp[i] <= 'f')
			uRet |= (szTemp[i] - 'a' + 10);
		else if (szTemp[i] >= 'A' && szTemp[i] <= 'F')
			uRet |= (szTemp[i] - 'A' + 10);
		else
			uRet |= 0;
	}

	return uRet;
}

unsigned long cl_atoulong(const char* pszNum)
{
	if (pszNum[0] == '0' && (pszNum[1] == 'x' || pszNum[1] == 'X'))
		return cl_ahtoulong(pszNum);
	else
		return cl_atol(pszNum);
}


void cl_printhex(unsigned char uNum)
{
	static char szHex[] = "0123456789abcdef";

	cl_putchar(szHex[(uNum >> 4) & 0x0f]);
	cl_putchar(szHex[uNum & 0x0f]);
}
