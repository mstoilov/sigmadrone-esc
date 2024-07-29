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

#ifndef _CLEDITLINE_H_
#define _CLEDITLINE_H_

#ifdef __cplusplus
extern "C" {
#endif

void cl_term_enable_wrap();
void cl_term_disable_wrap();
int cl_editline(const char *pszPrompt, char *pszBuffer, unsigned int uSize, unsigned int uHistoryEntries);

#ifdef __cplusplus
}
#endif


#endif
