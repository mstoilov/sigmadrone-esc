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
#include "cdc_iface.h"
#include "pwm_generator.h"
#include "adc.h"
#include "drv8323.h"
#include "motordrive.h"
#include "motordrive.h"
#include "rexjson/rexjsonproperty.h"

extern rexjson::property *g_properties;
extern rexjson::property *g_config_properties;
extern CdcIface usb_cdc;
extern std::vector<MotorDrive*> g_motors;
extern Adc adc1;
extern Drv8323 drv1;

UartRpcServer::UartRpcServer()
	: rexjson::rpc_server<UartRpcServer>()
{
	add("get", &UartRpcServer::rpc_get_property);
	add("set", &UartRpcServer::rpc_set_property);
	add("getcfg", &UartRpcServer::rpc_get_config_property);
	add("setcfg", &UartRpcServer::rpc_set_config_property);
}

UartRpcServer::~UartRpcServer()
{

}

rexjson::value UartRpcServer::rpc_get_property(rexjson::array& params, rexjson::rpc_exec_mode mode)
{
	static std::string prefix = "";
	std::ostringstream oss;
	static unsigned int types[] = {rexjson::rpc_str_type};
	static const char *help_msg = "Get property.\r\n\r\n";

	if (mode != rexjson::execute) {
		g_properties->enumerate(*g_properties, prefix, [&](const std::string& path, rexjson::property& prop)->void
				{
					oss << path << " : " << prop.get().to_string() << "\r\n";
				});
		std::string help = help_msg;
		help += oss.str();
		return noexec(params, mode, types, ARRAYSIZE(types), help);
	}
	verify_parameters(params, types, ARRAYSIZE(types));
	std::string path = params[0].get_str();
	return g_properties->navigate(path.substr(prefix.size())).get();
}

rexjson::value UartRpcServer::rpc_set_property(rexjson::array& params, rexjson::rpc_exec_mode mode)
{
	static std::string prefix = "";
	std::ostringstream oss;
	static unsigned int types[] = {rexjson::rpc_str_type, rexjson::rpc_str_type|rexjson::rpc_int_type|rexjson::rpc_real_type|rexjson::rpc_bool_type};
	static const char *help_msg = "Set property.\r\n";

	if (mode != rexjson::execute) {
		g_properties->enumerate(*g_properties, prefix, [&](const std::string& path, rexjson::property& prop)->void
				{
					oss << path << " : " << prop.get().to_string() << "\r\n\r\n";
				});
		std::string help = help_msg;
		help += oss.str();
		return noexec(params, mode, types, ARRAYSIZE(types), help);
	}
	verify_parameters(params, types, ARRAYSIZE(types));
	std::string path = params[0].get_str();
	g_properties->navigate(path.substr(prefix.size())).set(params[1]);
	return g_properties->navigate(path.substr(prefix.size())).get();
}


rexjson::value UartRpcServer::rpc_get_config_property(rexjson::array& params, rexjson::rpc_exec_mode mode)
{
	static std::string prefix = "";
	std::ostringstream oss;
	static unsigned int types[] = {rexjson::rpc_str_type};
	static const char *help_msg = "Get Configuration property.\r\n\r\n";

	if (mode != rexjson::execute) {
		g_config_properties->enumerate(*g_config_properties, prefix, [&](const std::string& path, rexjson::property& prop)->void
				{
					oss << path << " : " << prop.get().to_string() << "\r\n";
				});
		std::string help = help_msg;
		help += oss.str();
		return noexec(params, mode, types, ARRAYSIZE(types), help);
	}
	verify_parameters(params, types, ARRAYSIZE(types));
	std::string path = params[0].get_str();
	return g_config_properties->navigate(path.substr(prefix.size())).get();
}

rexjson::value UartRpcServer::rpc_set_config_property(rexjson::array& params, rexjson::rpc_exec_mode mode)
{
	static std::string prefix = "";
	std::ostringstream oss;
	static unsigned int types[] = {rexjson::rpc_str_type, rexjson::rpc_str_type|rexjson::rpc_int_type|rexjson::rpc_real_type|rexjson::rpc_bool_type};
	static const char *help_msg = "Set Configuration property.\r\n\r\n";

	if (mode != rexjson::execute) {
		g_config_properties->enumerate(*g_config_properties, prefix, [&](const std::string& path, rexjson::property& prop)->void
				{
					oss << path << " : " << prop.get().to_string() << "\r\n";
				});
		std::string help = help_msg;
		help += oss.str();
		return noexec(params, mode, types, ARRAYSIZE(types), help);
	}
	verify_parameters(params, types, ARRAYSIZE(types));
	std::string path = params[0].get_str();
	g_config_properties->navigate(path.substr(prefix.size())).set(params[1]);
	return g_config_properties->navigate(path.substr(prefix.size())).get();
}


