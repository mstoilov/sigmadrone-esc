#include <assert.h>
#include <string.h>
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
	while (1) {
		if ((elret = cl_editline("sigmadrive # ", szBuffer, sizeof(szBuffer), 5)) > 0) {
			printf("\r\n");
			assert(elret == (int)strlen(szBuffer));
			try {
				std::string str(szBuffer, 0, elret);
				std::cout << rpc_server.call(str).write(true, true, 4, 4) << std::endl;
			} catch (std::runtime_error& e) {
				std::cout << e.what() << std::endl;
			}
		}
		printf("\r\n");
	}

}
