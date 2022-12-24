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
 */

#ifndef _RPCCLIENTUART_H_
#define _RPCCLIENTUART_H_

#include <termios.h>
#include "rexjson/rexjson++.h"

class rpc_client_uart {
public:
	rpc_client_uart(const std::string& filename, speed_t speed = B115200, size_t usec = 1500000, const std::string& jsonrpc_version = "1.0");
	~rpc_client_uart();
	std::string json_rpc_request(const std::string& json);
	rexjson::value call(const std::string& method, const rexjson::array& params);

	rexjson::value call(
		const std::string& method
	);

	rexjson::value call(
		const std::string& method,
		const rexjson::value& val1
	);

	rexjson::value call(
		const std::string& method,
		const rexjson::value& val1,
		const rexjson::value& val2
	);

	rexjson::value call(
		const std::string& method,
		const rexjson::value& val1,
		const rexjson::value& val2,
		const rexjson::value& val3
	);

	rexjson::value call(
		const std::string& method,
		const rexjson::value& val1,
		const rexjson::value& val2,
		const rexjson::value& val3,
		const rexjson::value& val4
	);

	int get_fd() { return fd_; }

public:
	std::string response_;

protected:
	int readtimeout(char *buf, size_t size);
	int readcache(char *buf, size_t size);
	void request(const std::string& str);
	std::string response();

protected:
	int fd_;
	size_t timeout_;
	std::string jsonrpc_version_;
	size_t serial_;
	char cache_[1024];
	char *cache_head_, *cache_tail_;
};

#endif /* _RPCCLIENTUART_H_ */
