/*
* Examples:
*
* MacOS:
* # ./bin/sigmadrive-tool -d /dev/cu.usbserial-A5026YP3 help
*
* Linux:
* ./bin/sigmadrive-tool help
* ./bin/sigmadrive-tool help get
* ./bin/sigmadrive-tool -r help get
*
* # ./bin/sigmadrive-tool -r get axis1.drive.Rencest
* {"id":"noid","jsonrpc":"1.0","method":"get","params":["axis1.drive.Rencest"]}
* {"id":"noid","result":19997925}
*/


#include <iostream>

#include "rpcclientuart.h"
#include "cmdargs/cmdargs.h"
#include "rexjson/rexjson++.h"


static cmd_arg_spec g_argspec[] = {
		{"help",			"h",	"Display this help", CMD_ARG_BOOL},
		{"dev",				"d",	"Uart Device", CMD_ARG_STRING},
		{"id",				"i",	"Request id", CMD_ARG_STRING},
		{"response",		"r",	"Display the RPC response, not just the result", CMD_ARG_BOOL},

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
			std::string strres;
			std::string req = create_rpc_request(args.get_value("id", "noid"), args.get_commands());
			std::cout << req << std::endl;
			strres = uart.json_rpc_request(req);
			if (!args.get_value("response").empty()) {
				std::cout << strres << std::endl;
			} else {
				rexjson::value res = rexjson::read(strres);
				std::cout << res["result"] << std::endl;
			}
		}
	} catch (std::exception& e) {
		std::cout << "Error: " << e.what() << std::endl;
	}

	return 0;
}
