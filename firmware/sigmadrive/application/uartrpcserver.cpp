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
#include "torque_loop.h"
#include "adc.h"
#include "drv8323.h"
#include "motordrive.h"
#include "motordrive.h"
#include "rexjsonrpc/property.h"

extern rexjson::property *g_properties;
extern CdcIface usb_cdc;
extern std::vector<MotorDrive*> g_motors;
extern Adc adc1;
extern Drv8323 drv1;
extern TorqueLoop tql;

UartRpcServer::UartRpcServer()
    : rexjson::rpc_server<UartRpcServer>()
{
    add("adc_injswstart", &UartRpcServer::rpc_adc_injswstart);
    add("drv_calibration", &UartRpcServer::rpc_drv_calibration);
    add("drv_csagain", &UartRpcServer::rpc_drv_csagain);
    add("get", &UartRpcServer::rpc_get_property);
    add("set", &UartRpcServer::rpc_set_property);
}

UartRpcServer::~UartRpcServer()
{

}

rexjson::value UartRpcServer::rpc_adc_injswstart(rexjson::array& params, rexjson::rpc_exec_mode mode)
{
    static unsigned int types[] = {rexjson::rpc_null_type};
    static const char *help_msg = R"desc(
adc_injswstart
Start ADC injected conversion.

Arguments:
)desc";

    if (mode != rexjson::execute) {
        return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
    }
    verify_parameters(params, types, ARRAYSIZE(types));

    adc1.InjectedSwTrig();
    return true;
}

rexjson::value UartRpcServer::rpc_drv_calibration(rexjson::array& params, rexjson::rpc_exec_mode mode)
{
    static unsigned int types[] = {rexjson::rpc_bool_type};
    static const char *help_msg = R"desc(
drv_calibration
Driver calibration enable/disable.

Arguments:
1. enable		(bool) true/false
)desc";

    if (mode != rexjson::execute) {
        return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
    }
    verify_parameters(params, types, ARRAYSIZE(types));

    params[0].get_bool() ? drv1.EnableCalibration() : drv1.DisableCalibration();
    return params[0].get_bool();
}

rexjson::value UartRpcServer::rpc_drv_csagain(rexjson::array& params, rexjson::rpc_exec_mode mode)
{
    static unsigned int types[] = {rexjson::rpc_int_type|rexjson::rpc_null_type};
    static const char *help_msg = R"desc(
drv_csagain
Get/Set Driver CSA gain.

Arguments:
1. gain		(int)  0, 1, 2, 3, 4  (x5VV, x10VV, x20VV, x40VV)
)desc";

    if (mode != rexjson::execute) {
        return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
    }
    verify_parameters(params, types, ARRAYSIZE(types));
    if (params[0].type() == rexjson::int_type) {
        drv1.SetCSAGain(params[0].get_int());
    }
    return drv1.GetCSAGain();
}

rexjson::value UartRpcServer::rpc_get_property(rexjson::array& params, rexjson::rpc_exec_mode mode)
{
    static std::string prefix = "";
    std::ostringstream oss;
    static unsigned int types[] = {rexjson::rpc_str_type};
    static const char *help_msg = R"desc(
Get property.

)desc";

    if (mode != rexjson::execute) {
        g_properties->enumerate_children(prefix, [&](const std::string& path, rexjson::property& prop)->void
                {
                    oss << path << " : " << prop.get_prop().to_string() << std::endl;
                });
        std::string help = help_msg;
        help += oss.str();
        return noexec(params, mode, types, ARRAYSIZE(types), help);
    }
    verify_parameters(params, types, ARRAYSIZE(types));
    std::string path = params[0].get_str();
    return g_properties->navigate(path.substr(prefix.size())).get_prop();
}

rexjson::value UartRpcServer::rpc_set_property(rexjson::array& params, rexjson::rpc_exec_mode mode)
{
    static std::string prefix = "";
    std::ostringstream oss;
    static unsigned int types[] = {rexjson::rpc_str_type, rexjson::rpc_str_type|rexjson::rpc_int_type|rexjson::rpc_real_type|rexjson::rpc_bool_type};
    static const char *help_msg = R"desc(
Set property.

)desc";

    if (mode != rexjson::execute) {
        g_properties->enumerate_children(prefix, [&](const std::string& path, rexjson::property& prop)->void
                {
                    oss << path << " : " << prop.get_prop().to_string() << std::endl;
                });
        std::string help = help_msg;
        help += oss.str();
        return noexec(params, mode, types, ARRAYSIZE(types), help);
    }
    verify_parameters(params, types, ARRAYSIZE(types));
    std::string path = params[0].get_str();
    g_properties->navigate(path.substr(prefix.size())).set_prop(params[1]);
    return g_properties->navigate(path.substr(prefix.size())).get_prop();
}
