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
#include "cmsis_os2.h"
#include "ryno/runtime.h"

char cl_heap[4096];
ryno::RunTime rt;


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
void RunRynoCommandTask(void *argument)
{
	*_impure_ptr = *_impure_data_ptr;

	cl_mem_init(cl_heap, sizeof(cl_heap), 100);
	cl_history_init();
	char szBuffer[2048];
	int elret;

	ryno::Initialize(rt);

	/*
	* wait for input from the terminal.
	*/
	std::cout << "\r\n";
	while (1) {
		if ((elret = cl_editline("sigmadrive # ", szBuffer, sizeof(szBuffer), 15)) > 0) {
			printf("\r\n");
			assert(elret == (int)strlen(szBuffer));
			try {
				std::string str(szBuffer, 0, elret);
				str += ";";
				ryno::RyPointer ret = rt.Exec(str, false);
				if (!ret->IsNil())
					std::cout << ret->Repr() << std::endl;

			} catch (std::runtime_error& e) {
				std::cout << e.what() << "\r\n";
			}
		}
		printf("\r\n");
	}
}

