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
#include <iostream>
#include "clientapp.h"


client_app::client_app(const cmd_args& args)
	: args_(args)
{
}

client_app::~client_app()
{
}

int client_app::run(int argc, const char *argv[])
{
	std::vector<std::string> parsed_cmd_line = parse_command_line(argc, argv);
	if (!parsed_cmd_line.size())
		throw std::runtime_error("no RPC method");

	std::string method = parsed_cmd_line[0];
	std::vector<std::string> params(parsed_cmd_line.size() - 1);
	std::copy(parsed_cmd_line.begin() + 1, parsed_cmd_line.end(), params.begin());
	std::cout << create_rpc_request(method, params) << std::endl;
	return 0;
}

std::vector<std::string> client_app::parse_command_line(int argc, const char *argv[])
{
	std::vector<std::string> ret;

	for (int i = 1; i < argc; i++) {
		std::string arg(argv[i]);
		if (arg[0] != '-') {
			ret.push_back(arg);
		}
	}
	return ret;
}

std::string client_app::create_rpc_request(const std::string& method, const std::vector<std::string>& params)
{
	rexjson::object rpc_request;
	rexjson::array parameters;

	rpc_request["jsonrpc"] = "1.0";
	rpc_request["id"] = "clientid";
	rpc_request["method"] = rexjson::value(method);

	for (size_t i = 0; i < params.size(); i++) {
		if (i < params.size()) {
			rexjson::value val;
			try {
				val = rexjson::read(params[i]);
				parameters.push_back(val);
			} catch (std::exception &e) {
				val = params[i];
				parameters.push_back(val);
			}
		} else {

		}
	}
	rpc_request["params"] = parameters;
	return rexjson::write(rpc_request);
}

std::string client_app::create_rpc_specrequest(const std::string& method)
{
	rexjson::object rpc_request;
	rexjson::array parameters;

	rpc_request["jsonrpc"] = "1.0";
	rpc_request["id"] = "clientid";
	rpc_request["method"] = "spec";
	parameters.push_back(method);
	rpc_request["params"] = parameters;
	return rexjson::write(rpc_request);
}
