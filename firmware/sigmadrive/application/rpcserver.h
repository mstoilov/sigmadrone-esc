/*
 *  Sigmadrone
 *  Copyright (c) 2013-2019 The Sigmadrone Developers
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
#ifndef _RPCSERVER_H_
#define _RPCSERVER_H_

#include <string>
#include <map>
#include <stdexcept>
#include <iostream>
#include "rexjson++.h"

#ifndef ARRAYSIZE
#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

enum rpc_error_code
{
    // Standard JSON-RPC 2.0 errors
    RPC_INVALID_REQUEST  = -32600,
    RPC_METHOD_NOT_FOUND = -32601,
    RPC_INVALID_PARAMS   = -32602,
    RPC_INTERNAL_ERROR   = -32603,
    RPC_PARSE_ERROR      = -32700,

    // General application defined errors
    RPC_MISC_ERROR                  = -1,  // std::exception thrown in command handling
};

struct rpc_server_dummy { };
struct rpc_connection_dummy { };

template<typename T = rpc_server_dummy, typename R = rpc_connection_dummy>
class rpc_server
{
public:
	/*
	 * The rpc types correspond directly to the json value types like obj_type, array_type ...
	 * but they have values 2 pow n, so we can store them in unsigned int like (rpc_str_type|rpc_null_type)
	 * Then we use a vector of these bit masks to specify the parameter types we expect for
	 * the rpc calls.
	 *
	 * get_rpc_type will convert rexjson::value_type to one of the rpc types.
	 *
	 * create_json_spec will convert the rpc_type bit mask to an integer or array of integers
	 * correspoinding to rexjson value_type(s)
	 *
	 * rpc_types will convert json spec to a std::vector of rpc_type_ bitmasks.
	 *
	 */
	static const unsigned int rpc_null_type = 1;
	static const unsigned int rpc_obj_type = 2;
	static const unsigned int rpc_array_type = 4;
	static const unsigned int rpc_str_type = 8;
	static const unsigned int rpc_bool_type = 16;
	static const unsigned int rpc_int_type = 32;
	static const unsigned int rpc_real_type = 64;
	enum rpc_exec_mode {
		execute = 0, 	// normal execution
		spec,			// produce machine readable parameter specification
		helpspec, 		// produce a human readable parameter specification
		help, 			// produce a help message
	};
	typedef rexjson::value (T::*rpc_method_type)(R connection, rexjson::array& params, rpc_exec_mode mode);
	typedef std::map<std::string, rpc_method_type> method_map_type;

	rpc_server()
	{
		add("help", &rpc_server::rpc_help);
		add("spec", &rpc_server::rpc_spec);
	}
	~rpc_server()
	{

	}

	rexjson::value rpc_spec(R connection, rexjson::array& params, rpc_exec_mode mode)
	{
		static unsigned int types[] = { rpc_str_type };
		if (mode != execute) {
			if (mode == spec)
				return create_json_spec(types, ARRAYSIZE(types));
			if (mode == helpspec)
				return create_json_helpspec(types, ARRAYSIZE(types));
			return
					"spec <\"name\">\n"
					"\nGet the spec for the specified rpc name.\n"
					"\nArguments:\n"
					"1. \"name\"     (string, required) The name of of the rpc method to get the spec on\n"
					"\nResult:\n"
					"\"json\"     (string) The rpc call spec in json\n";
		}

		verify_parameters(params, types, ARRAYSIZE(types));
		rexjson::array ignored;
		return call_method_name(connection, params[0], ignored, spec);
	}

	rexjson::value rpc_help(R connection, rexjson::array& params, rpc_exec_mode mode)
	{
		static unsigned int types[] = { (rpc_str_type | rpc_null_type) };
		if (mode != execute) {
			if (mode == spec)
				return create_json_spec(types, ARRAYSIZE(types));
			if (mode == helpspec)
				return create_json_helpspec(types, ARRAYSIZE(types));
			return
					"help [\"command\"]\n"
					"\nList all commands, or get help for a specified command.\n"
					"\nArguments:\n"
					"1. \"command\"     (string, optional) The command to get help on\n"
					"\nResult:\n"
					"\"text\"     (string) The help text\n";
		}

		verify_parameters(params, types, ARRAYSIZE(types));
		if (params[0].type() == rexjson::null_type) {
			std::string result;
			for (typename method_map_type::const_iterator it = map_.begin(); it != map_.end(); it++) {
				rexjson::array ignored;
				std::string ret = call_method_name(connection, rexjson::value(it->first), ignored, help).get_str();
				result += ret.substr(0, ret.find('\n')) + "\n";
			}
			return result;
		}
		rexjson::array ignored;
		return call_method_name(connection, params[0], ignored, help);
	}


	static unsigned int get_rpc_type(rexjson::value_type value_type)
	{
		static const unsigned int rpc_types[] = {rpc_null_type, rpc_obj_type, rpc_array_type, rpc_str_type, rpc_bool_type, rpc_int_type, rpc_real_type};
		return rpc_types[value_type];
	}

	static rexjson::value create_json_spec(unsigned int *arr, size_t n)
	{
		rexjson::array params;

		for (size_t i = 0; i < n; i++) {
			rexjson::array param;
			if (arr[i] & rpc_obj_type)
				param.push_back(rexjson::obj_type);
			if (arr[i] & rpc_array_type)
				param.push_back(rexjson::array_type);
			if (arr[i] & rpc_str_type)
				param.push_back(rexjson::str_type);
			if (arr[i] & rpc_bool_type)
				param.push_back(rexjson::bool_type);
			if (arr[i] & rpc_int_type)
				param.push_back(rexjson::int_type);
			if (arr[i] & rpc_real_type)
				param.push_back(rexjson::real_type);
			if (arr[i] & rpc_null_type)
				param.push_back(rexjson::null_type);
			params.push_back((param.size() > 1) ? param : param[0]);
		}
		return params;
	}

	static std::vector<unsigned int> rpc_types(const rexjson::array& spec)
	{
		std::vector<unsigned int> ret;

		for (size_t i = 0; i < spec.size(); i++) {
			if (spec[i].get_type() == rexjson::array_type) {
				unsigned int rpc_types = 0;
				for (size_t j = 0; j < spec[i].get_array().size(); j++) {
					rpc_types |= get_rpc_type((rexjson::value_type)spec[i].get_array()[j].get_int());
				}
				ret.push_back(rpc_types);
			} else {
				ret.push_back(get_rpc_type((rexjson::value_type)spec[i].get_int()));
			}
		}
		return ret;
	}

	static std::vector<unsigned int> rpc_types(const std::string strspec)
	{
		rexjson::value jsonspec;

		jsonspec.read(strspec);
		return rpc_types(jsonspec.get_array());
	}

	static rexjson::value create_json_helpspec(unsigned int *arr, size_t n)
	{
		rexjson::value ret = create_json_spec(arr, n);
		convert_types_to_strings(ret);
		return ret;
	}

	static rexjson::object create_rpc_error(
			rpc_error_code code,
			const std::string& message)
	{
		rexjson::object error;
		error["code"] = code;
		error["message"] = message;
		return error;
	}

	void add(const std::string& name, rpc_method_type method)
	{
		if (name.empty())
			throw std::runtime_error("rpc_server::add, invalid name parameter");
		if (!method)
			throw std::runtime_error("rpc_server::add, invalid method parameter");
		map_[name] = method;
	}

	rexjson::value call_method_name(R connection, const rexjson::value& methodname, rexjson::array& params, rpc_exec_mode mode = execute)
	{
		if (methodname.get_type() != rexjson::str_type)
			throw create_rpc_error(RPC_INVALID_REQUEST, "method must be a string");
		typename method_map_type::const_iterator method_entry = map_.find(methodname.get_str());
		if (method_entry == map_.end())
			throw create_rpc_error(RPC_METHOD_NOT_FOUND, "method not found");
		return (static_cast<T*>(this)->*(method_entry->second))(connection, params, mode);
	}

	rexjson::value call(R connection, const rexjson::value& val, rpc_exec_mode mode = execute)
	{
		rexjson::object ret;
		rexjson::value result;
		rexjson::value error;
		rexjson::value id;
		rexjson::array params;

		try {
			if (val.get_type() != rexjson::obj_type)
				throw create_rpc_error(RPC_PARSE_ERROR, "top-level object parse error");
			rexjson::object::const_iterator params_it = val.get_obj().find("params");
			if (params_it != val.get_obj().end() && params_it->second.type() != rexjson::array_type)
				throw create_rpc_error(RPC_INVALID_REQUEST, "params must be an array");
			if (params_it != val.get_obj().end())
				params = params_it->second.get_array();
			rexjson::object::const_iterator id_it = val.get_obj().find("id");
			if (id_it != val.get_obj().end())
				id = id_it->second;
			rexjson::object::const_iterator method_it = val.get_obj().find("method");
			if (method_it == val.get_obj().end())
				throw create_rpc_error(RPC_INVALID_REQUEST, "missing method");
			result = call_method_name(connection, method_it->second, params, mode);
		} catch (rexjson::object& e) {
			error = e;
		} catch (std::exception& e) {
			rexjson::object errobj;
			errobj["message"] = e.what();
			errobj["code"] = RPC_MISC_ERROR;
			error = errobj;
		}
		ret["result"] = result;
		ret["id"] = id;
		ret["error"] = error;
		return ret;
	}

	rexjson::value call(R connection, const std::string& name, const rexjson::value& id, const rexjson::array& params, rpc_exec_mode mode = execute)
	{
		rexjson::object req;
		req["id"] = id;
		req["method"] = rexjson::value(name);
		req["params"] = rexjson::value(params);
		return call(connection, rexjson::value(req), mode);
	}

	rexjson::value call(R connection, const std::string& request, rpc_exec_mode mode = execute)
	{
		rexjson::object ret;
		rexjson::value result;
		rexjson::value error;
		rexjson::value id;

		try {
			rexjson::value val;
			val.read(request);
			return call(connection, val, mode);
		} catch (rexjson::object& e) {
			error = e;
		} catch (std::exception& e) {
			rexjson::object errobj;
			errobj["message"] = e.what();
			errobj["code"] = RPC_MISC_ERROR;
			error = errobj;
		}
		ret["result"] = result;
		ret["id"] = id;
		ret["error"] = error;
		return ret;
	}

protected:
	static void convert_types_to_strings(rexjson::value& val)
	{
		if (val.get_type() == rexjson::int_type && val.get_int() >= rexjson::obj_type && val.get_int() <= rexjson::null_type) {
			std::string strtypename = val.get_typename();
			val = strtypename;
		} else if (val.get_type() == rexjson::array_type) {
			for (size_t i = 0; i < val.get_array().size(); i++) {
				convert_types_to_strings(val.get_array()[i]);
			}
		}
	}

	static void verify_parameters(rexjson::array& params, unsigned int *types, size_t n)
	{
		params.resize(n);
		for (size_t i = 0; i < n; i++) {
			rexjson::value_type value_type = params[i].get_type();
			if ((get_rpc_type(value_type) & types[i]) == 0) {
				throw create_rpc_error(RPC_INVALID_PARAMS, "Invalid parameter: '" + params[i].write(false) + "'");
			}
		}
	}

protected:
	method_map_type map_;
};

#endif /* _RPCSERVER_H_ */
