/*
 *  Sigmadrone
 *  Copyright (c) 2013-2015 The Sigmadrone Developers
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
 *  Svetoslav Vassilev <svassilev@sigmadrone.org>
 */
#ifndef CLIENTAPP_H
#define CLIENTAPP_H

#include <string>
#include <memory.h>

#include "cmdargs/cmdargs.h"
#include "rexjson/rexjson++.h"

class client_app
{
public:
	client_app(const cmd_args& args);
	~client_app();
	int run(int argc, const char *argv[]);
	std::vector<std::string> parse_command_line(int argc, const char *argv[]);
	std::string create_rpc_request(const std::string& method, const std::vector<std::string>& params);
	std::string create_rpc_specrequest(const std::string& method);

protected:
	const cmd_args& args_;
};


#endif // APPLICATION_CLIENT_H
