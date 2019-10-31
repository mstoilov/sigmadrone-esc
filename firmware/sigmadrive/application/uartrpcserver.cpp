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
#include "servo_drive.h"
#include "property.h"

extern rexjson::property *g_properties;
extern CdcIface usb_cdc;
extern std::vector<IServoDrive*> g_motors;
extern Adc adc1;
extern Drv8323 drv1;
extern TorqueLoop tql;

UartRpcServer::UartRpcServer()
	: rpc_server<UartRpcServer>()
{
	add("pwm_timings", &UartRpcServer::rpc_pwm_timings);
	add("pwm_start", &UartRpcServer::rpc_pwm_start);
	add("pwm_stop", &UartRpcServer::rpc_pwm_stop);
	add("adc_injswstart", &UartRpcServer::rpc_adc_injswstart);
	add("adc_injhistory", &UartRpcServer::rpc_adc_injhistory);
	add("adc_injbias", &UartRpcServer::rpc_adc_injbias);
	add("adc_injdata", &UartRpcServer::rpc_adc_injdata);
	add("adc_injcurrent", &UartRpcServer::rpc_adc_injcurrent);
	add("drv_calibration", &UartRpcServer::rpc_drv_calibration);
	add("drv_csagain", &UartRpcServer::rpc_drv_csagain);
	add("getset_property", &UartRpcServer::rpc_getset_property);
}

UartRpcServer::~UartRpcServer()
{

}


rexjson::value UartRpcServer::rpc_pwm_timings(rexjson::array& params, rpc_exec_mode mode)
{
	static unsigned int types[] = {rpc_int_type, rpc_array_type|rpc_null_type};
	static const char *help_msg = R"desc(
pwm_timings
Get/Set OC Values.

Arguments:
1. motor		(int) Motor index, starting from 0
2. values		(array) OC values for the different channels. 
)desc";

	if (mode != execute) {
		return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
	}
	verify_parameters(params, types, ARRAYSIZE(types));

	uint32_t motor = params[0].get_int();
	if (motor >= g_motors.size())
		throw std::range_error("Invalid motor index");
	if (params[1].type() == rexjson::array_type) {
		for (uint32_t i = 0; i < params[1].get_array().size(); i++)
			g_motors[motor]->GetPwmGenerator()->SetTiming(i + 1, params[1].get_array()[i].get_int());
	}

	rexjson::array ret;
	ret.push_back((int)g_motors[motor]->GetPwmGenerator()->GetTiming(1));
	ret.push_back((int)g_motors[motor]->GetPwmGenerator()->GetTiming(2));
	ret.push_back((int)g_motors[motor]->GetPwmGenerator()->GetTiming(3));
	ret.push_back((int)g_motors[motor]->GetPwmGenerator()->GetTiming(4));
	return ret;
}

rexjson::value UartRpcServer::rpc_pwm_start(rexjson::array& params, rpc_exec_mode mode)
{
	static unsigned int types[] = {rpc_int_type};
	static const char *help_msg = R"desc(
pwm_start
Start the PWM Generation.

Arguments:
1. motor		(int) Motor index, starting from 0
)desc";

	if (mode != execute) {
		return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
	}
	verify_parameters(params, types, ARRAYSIZE(types));

	uint32_t motor = params[0].get_int();
	if (motor >= g_motors.size())
		throw std::range_error("Invalid motor index");
	g_motors[motor]->Start();
	return g_motors[motor]->IsStarted();
}

rexjson::value UartRpcServer::rpc_pwm_stop(rexjson::array& params, rpc_exec_mode mode)
{
	static unsigned int types[] = {rpc_int_type};
	static const char *help_msg = R"desc(
pwm_stop
Stop the PWM Generation.

Arguments:
1. motor		(int) Motor index, starting from 0
)desc";

	if (mode != execute) {
		return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
	}
	verify_parameters(params, types, ARRAYSIZE(types));

	uint32_t motor = params[0].get_int();
	if (motor >= g_motors.size())
		throw std::range_error("Invalid motor index");
	g_motors[motor]->Stop();
	return g_motors[motor]->IsStarted();

}

rexjson::value UartRpcServer::rpc_adc_injswstart(rexjson::array& params, rpc_exec_mode mode)
{
	static unsigned int types[] = {rpc_null_type};
	static const char *help_msg = R"desc(
adc_injswstart
Start ADC injected conversion.

Arguments:
)desc";

	if (mode != execute) {
		return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
	}
	verify_parameters(params, types, ARRAYSIZE(types));

	adc1.InjectedSwTrig();
	return true;
}

rexjson::value UartRpcServer::rpc_adc_injhistory(rexjson::array& params, rpc_exec_mode mode)
{
	static unsigned int types[] = {rpc_int_type};
	static const char *help_msg = R"desc(
adc_injhistory
Get injected data for the specified channel

Arguments:
1. channel		(int) ADC injected data channel.
)desc";

	if (mode != execute) {
		return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
	}
	verify_parameters(params, types, ARRAYSIZE(types));

	uint32_t channel = params[0].get_int();
	if (channel >= 3)
		throw std::range_error("Invalid channel");

	rexjson::array ret;
	for (size_t i = 0; i < Adc::ADC_HISTORY_SIZE; i++)
		ret.push_back((int)adc1.injhistory_[i][channel]);

	return ret;
}

rexjson::value UartRpcServer::rpc_adc_injbias(rexjson::array& params, rpc_exec_mode mode)
{
	static unsigned int types[] = {rpc_null_type};
	static const char *help_msg = R"desc(
adc_bias
Get bias.

Arguments:
	none
)desc";

	if (mode != execute) {
		return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
	}
	verify_parameters(params, types, ARRAYSIZE(types));

	rexjson::array ret;
	for (size_t i = 0; i < sizeof(adc1.bias_)/sizeof(adc1.bias_[0]); i++)
		ret.push_back((int)adc1.bias_[i]);
	return ret;
}

rexjson::value UartRpcServer::rpc_adc_injdata(rexjson::array& params, rpc_exec_mode mode)
{
	static unsigned int types[] = {rpc_null_type};
	static const char *help_msg = R"desc(
adc_injdata
Get injected data.

Arguments:
	none
)desc";

	if (mode != execute) {
		return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
	}
	verify_parameters(params, types, ARRAYSIZE(types));

	rexjson::array ret;
	for (size_t i = 0; i < sizeof(adc1.injdata_)/sizeof(adc1.injdata_[0]); i++)
		ret.push_back((int)adc1.injdata_[i]);
	return ret;
}

rexjson::value UartRpcServer::rpc_adc_injcurrent(rexjson::array& params, rpc_exec_mode mode)
{
	static unsigned int types[] = {rpc_null_type};
	static const char *help_msg = R"desc(
adc_injcurrent
Get injected current.

Arguments:
	none
)desc";

	if (mode != execute) {
		return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
	}
	verify_parameters(params, types, ARRAYSIZE(types));

	float csa_gain = drv1.GetCSAGainValue();
	float Rsense = 0.010f;
	float totalCurrent = 0.0f;

	rexjson::array ret;
	for (size_t i = 0; i < sizeof(adc1.injdata_)/sizeof(adc1.injdata_[0]); i++) {
		float current = (float)(((float)adc1.bias_[i] - (float)adc1.injdata_[i]) / 1000.0f / (csa_gain * Rsense));
		totalCurrent += current;
		ret.push_back(current);
	}
	ret.push_back(totalCurrent);
	return ret;
}

rexjson::value UartRpcServer::rpc_drv_calibration(rexjson::array& params, rpc_exec_mode mode)
{
	static unsigned int types[] = {rpc_bool_type};
	static const char *help_msg = R"desc(
drv_calibration
Driver calibration enable/disable.

Arguments:
1. enable		(bool) true/false
)desc";

	if (mode != execute) {
		return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
	}
	verify_parameters(params, types, ARRAYSIZE(types));

	params[0].get_bool() ? drv1.EnableCalibration() : drv1.DisableCalibration();
	return params[0].get_bool();
}

rexjson::value UartRpcServer::rpc_drv_csagain(rexjson::array& params, rpc_exec_mode mode)
{
	static unsigned int types[] = {rpc_int_type|rpc_null_type};
	static const char *help_msg = R"desc(
drv_csagain
Get/Set Driver CSA gain.

Arguments:
1. gain		(int)  0, 1, 2, 3, 4  (x5VV, x10VV, x20VV, x40VV)
)desc";

	if (mode != execute) {
		return noexec(params, mode, types, ARRAYSIZE(types), help_msg);
	}
	verify_parameters(params, types, ARRAYSIZE(types));
	if (params[0].type() == rexjson::int_type) {
		drv1.SetCSAGain(params[0].get_int());
	}
	return (int) drv1.GetCSAGain();
}

rexjson::value UartRpcServer::rpc_getset_property(rexjson::array& params, rpc_exec_mode mode)
{
	static std::string prefix = "props";
	std::ostringstream oss;
	static unsigned int types[] = {rpc_str_type, rpc_str_type|rpc_int_type|rpc_real_type|rpc_bool_type|rpc_null_type};
	static const char *help_msg = R"desc(
Get/Set property.

)desc";

	if (mode != execute) {
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
	if (params[1].type() != rexjson::null_type) {
		g_properties->navigate(path.substr(prefix.size())).set_prop(params[1]);
	}
	return g_properties->navigate(path.substr(prefix.size())).get_prop();
}
