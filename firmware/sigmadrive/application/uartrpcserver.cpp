/*
 *  Sigmadrive
 *  Copyright (c) 2013-2019 Martin Stoilov
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
 *
 */
#include <memory>
#include "uartrpcserver.h"


UartRpcServer::UartRpcServer()
	: rpc_server<UartRpcServer>()
{
	add("kp", &UartRpcServer::rpc_position_kp);
}

UartRpcServer::~UartRpcServer()
{

}

rexjson::value UartRpcServer::rpc_position_kp(rexjson::array& params, rpc_exec_mode mode)
{
	static unsigned int types[] = {rpc_real_type|rpc_int_type|rpc_null_type};
	if (mode != execute) {
		if (mode == spec)
			return create_json_spec(types, ARRAYSIZE(types));
		if (mode == helpspec)
			return create_json_helpspec(types, ARRAYSIZE(types));
		return
	            "sd_position_kp\n"
	            "\nGet/Set Kp for the PID controller."
				"\nIf the new coefficient is not specified, the current Kp will be returned."
				"\n"
				"Arguments:\n"
				"1. Kp          (real, optional) The Kp of the PID controller.\n"
				;
	}
	verify_parameters(params, types, ARRAYSIZE(types));
	if (params[0].type() != rexjson::null_type) {
		kp_ = params[0].get_real();
	}
	return kp_;
}
