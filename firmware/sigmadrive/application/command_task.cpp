#include <assert.h>
#include <string.h>
#include "unistd.h"
#include "command_task.h"
#include "ClEditLine.h"
#include "ClHistory.h"
#include "ClPort.h"
#include "rexjson++.h"
#include "uartrpcserver.h"

char cl_heap[4096];

extern "C"
void StartCommandTask(void *argument)
{
	cl_mem_init(cl_heap, sizeof(cl_heap), 100);
	cl_history_init();
	UartRpcServer rpc_server;
	char szBuffer[512];
	int elret;
	/*
	 * wait for input from the terminal.
	 */
//	getchar();
	while (1) {
		if ((elret = cl_editline("sigmadrive # ", szBuffer, sizeof(szBuffer), 15)) > 0) {
			printf("\r\n");
			assert(elret == (int)strlen(szBuffer));
			try {
				std::string str(szBuffer, 0, elret);
#if 0
				std::cout << rpc_server.call(str).write(true, true, 4, 4) << std::endl;
#else
				rexjson::value ret = rpc_server.call(str);
				if (ret["error"].get_type() != rexjson::null_type)
					std::cout << "ERROR: " << ret["error"]["message"].to_string() << std::endl;
				else
					std::cout << ret["result"].to_string() << std::endl;
#endif
			} catch (std::runtime_error& e) {
				std::cout << e.what() << std::endl;
			}
		}
		printf("\r\n");
	}

}
