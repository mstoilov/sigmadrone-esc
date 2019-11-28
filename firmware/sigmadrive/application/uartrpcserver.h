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

#ifndef UARTRPCSERVER_H_
#define UARTRPCSERVER_H_

#include "uart.h"
#include "rpcserver.h"

class UartRpcServer : public rexjson::rpc_server<UartRpcServer>
{
public:
	UartRpcServer();
	virtual ~UartRpcServer();

protected:
	rexjson::value rpc_pwm_timings(rexjson::array& params, rexjson::rpc_exec_mode mode = rexjson::execute);
	rexjson::value rpc_pwm_start(rexjson::array& params, rexjson::rpc_exec_mode mode = rexjson::execute);
	rexjson::value rpc_pwm_stop(rexjson::array& params, rexjson::rpc_exec_mode mode = rexjson::execute);
	rexjson::value rpc_adc_injswstart(rexjson::array& params, rexjson::rpc_exec_mode mode = rexjson::execute);
	rexjson::value rpc_adc_injhistory(rexjson::array& params, rexjson::rpc_exec_mode mode = rexjson::execute);
	rexjson::value rpc_adc_injdata(rexjson::array& params, rexjson::rpc_exec_mode mode = rexjson::execute);
	rexjson::value rpc_adc_injcurrent(rexjson::array& params, rexjson::rpc_exec_mode mode = rexjson::execute);
	rexjson::value rpc_adc_injbias(rexjson::array& params, rexjson::rpc_exec_mode mode = rexjson::execute);
	rexjson::value rpc_drv_calibration(rexjson::array& params, rexjson::rpc_exec_mode mode = rexjson::execute);
	rexjson::value rpc_drv_csagain(rexjson::array& params, rexjson::rpc_exec_mode mode = rexjson::execute);
	rexjson::value rpc_getset_property(rexjson::array& params, rexjson::rpc_exec_mode mode = rexjson::execute);

protected:
	float kp_ = 0.0;
};

#endif /* UARTRPCSERVER_H_ */
