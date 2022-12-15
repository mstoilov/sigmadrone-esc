#include <assert.h>
#include <string.h>
#include <sstream>
#include <iterator>
#include <algorithm>
#include "main.h"
#include "unistd.h"
#include "command_task.h"
#include "commandline/ClEditLine.h"
#include "commandline/ClHistory.h"
#include "commandline/ClPort.h"
#include "rexjson/rexjson++.h"
#include "uartrpcserver.h"
#include "cmsis_os2.h"

char cl_heap[4096];

extern UartRpcServer rpc_server;

std::string CommandToJson(std::string str)
{
	str.erase(str.begin(), std::find_if(str.begin(), str.end(),
					std::not1(std::ptr_fun<int, int>(std::isspace))));
	if (str.length() && str[0] == '{')
		return str;
	std::istringstream ss(str);
	std::istream_iterator<std::string> it(ss), eos;
	std::string json;
	if (it == eos)
		return json;
	json = "{\"method\" : ";
	json += "\"" + *it++ + "\"";
	json += ", \"params\" : [";
	for (size_t i = 0; it != eos; it++, i++) {
		std::string token = *it; //tokens[i];
		if (i != 0)
			json += ", ";
		if (std::isdigit(token[0])
			|| (token[0] == '-' && token.size() > 1 && std::isdigit(token[1]))
			|| token == "true"
			|| token == "false") {
			json += token;
		} else {
			json += "\"" + token + "\"";
		}
	}
	json += "]}";
	return json;
}


extern "C"
void RunCommandTask_experimental(void *argument)
{
	*_impure_ptr = *_impure_data_ptr;

	cl_mem_init(cl_heap, sizeof(cl_heap), 100);
	cl_history_init();
	char szBuffer[2048];
	int elret;

	while (1) {
		elret = cl_editline("sigmadrive # ", szBuffer, sizeof(szBuffer), 15);
		std::cout << "\r\n";

		if (elret > 0) {
			assert(elret == (int)strlen(szBuffer));
			try {
				std::cout << "You entered: " << szBuffer << "\r\n";

			} catch (std::runtime_error& e) {
				std::cout << e.what() << "\r\n";
			}
		}
	}
}


extern "C"
void RunCommandTask(void *argument)
{
	*_impure_ptr = *_impure_data_ptr;

	cl_mem_init(cl_heap, sizeof(cl_heap), 100);
	cl_history_init();
	char szBuffer[2048];
	int elret;
	/*
	 * wait for input from the terminal.
	 */
	while (1) {
		if ((elret = cl_editline("sigmadrive # ", szBuffer, sizeof(szBuffer), 15)) > 0) {
			printf("\r\n");
			assert(elret == (int)strlen(szBuffer));
			try {
				std::string str(szBuffer, 0, elret);
				rexjson::value ret = rpc_server.call(CommandToJson(str));
				if (ret["error"].get_type() != rexjson::null_type) {
					std::cout << "ERROR: " << ret["error"]["message"].to_string() << "\r\n";
				} else {
					if (ret["result"].get_type() == rexjson::obj_type || ret["result"].get_type() == rexjson::array_type)
						std::cout << ret["result"].write(false, true, 4, 4) << "\r\n";
					else
						std::cout << ret["result"].to_string() << "\r\n";
				}
			} catch (std::runtime_error& e) {
				std::cout << e.what() << "\r\n";
			}
		}
		printf("\r\n");
	}

}
