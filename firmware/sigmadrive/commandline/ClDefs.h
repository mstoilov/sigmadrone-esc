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

#ifndef _CLDEFS_H_
#define _CLDEFS_H_

#define NULL			((void*)0)
#define SLLOCHAR(c)		(((c) >= 'A' && (c) <= 'Z') ? (c) + ('a' - 'A'): (c))
#define SLHICHAR(c)		(((c) >= 'a' && (c) <= 'z') ? (c) - ('a' - 'A'): (c))
#define SLISDIGIT(c)	(((c) >= '0') && ((c) <= '9'))
#define SLISXDIGIT(c)	(ALISDIGIT(c) || (((c) >= 'a') && ((c) <= 'f')) || (((c) >= 'A') && ((c) <= 'F')))

#endif
