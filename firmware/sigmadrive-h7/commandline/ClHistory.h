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

#ifndef _CLHISTORY_H_
#define _CLHISTORY_H_

#if defined(__cplusplus)
extern "C" {
#endif


int cl_history_init();
int cl_history_add(const char *pszText);
void cl_history_head();
const char *cl_history_next();
const char *cl_history_prev();
int cl_history_remove_tail();
int cl_history_remove_head();
void cl_history_remove_all();
int cl_history_available_size();
int cl_history_size();
int cl_history_entries();
int cl_history_is_empty();

#if defined(__cplusplus)
}
#endif


#endif /* _CLHISTORY_H_ */
