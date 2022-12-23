#include <iostream>

#include "rpcclientuart.h"
#include "cmdargs/cmdargs.h"
#include "rexjson/rexjson++.h"


static cmd_arg_spec g_argspec[] = {
		{"help",			"h",	"Display this help", CMD_ARG_BOOL},
		{"dev",				"d",	"Uart Device", CMD_ARG_STRING},
		{"id",				"i",	"Request id", CMD_ARG_STRING},
};

std::string create_rpc_request(const std::string& id, const std::vector<std::string>& params)
{
	rexjson::object rpc_request;
	rexjson::array parameters;

	if (params.size() == 0)
		throw (std::runtime_error("Missing method name."));

	rpc_request["jsonrpc"] = "1.0";
	rpc_request["id"] = id.empty() ? "clientid" : id;
	rpc_request["method"] = rexjson::value(params[0]);

	for (size_t i = 1; i < params.size(); i++) {
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

std::vector<std::string> parse_command_line(int argc, const char *argv[])
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


int main(int argc, const char *argv[])
{
	cmd_args args;

	try {
		args.add_specs(g_argspec, sizeof(g_argspec)/sizeof(g_argspec[0]));
		args.parse_command_line(argc, argv);
		if (!args.get_value("help").empty()) {
			std::cout << argv[0] << " <options>" << std::endl;
			std::cout << args.get_help_message() << std::endl;
		} else {
			rpc_client_uart uart(args.get_value("dev", "/dev/ttyUSB1"));
			std::string res;
			std::string req = create_rpc_request(args.get_value("id", "noid"), parse_command_line(argc, argv));
			std::cout << req << std::endl;
			res = uart.json_rpc_request(req);
			std::cout << res << std::endl;

		}
	} catch (std::exception& e) {
		std::cout << "Error: " << e.what() << std::endl;
	}

	return 0;
}
