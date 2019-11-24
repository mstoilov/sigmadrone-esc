/*
 * scheduler.h
 *
 *  Created on: Nov 22, 2019
 *      Author: mstoilov
 */

#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "cmsis_os.h"
#include "task.h"

#include <functional>
#include <deque>

class Scheduler {
public:
	Scheduler();
	virtual ~Scheduler();

	void StartDispatcherThread();
	void Abort();
	void Run();
	void AddTask(const std::function<void(void)>& task);
	void RunSchedulerLoop();
	void SignalThreadUpdate();
	void SignalThreadTask();
	void SignalThreadAbort();

	bool WaitUpdate(uint32_t timeout_msec);
	bool WaitTask(uint32_t timeout_msec);
	bool WaitAbort(uint32_t timeout_msec);

	enum Signals {
		THREAD_SIGNAL_TASK = (1u << 0),
        THREAD_SIGNAL_UPDATE = (1u << 1),
        THREAD_SIGNAL_ABORT = (1u << 2)
    };

	uint32_t wait_timeout_ = (uint32_t)-1;
	osThreadId scheduler_thread_id_ = 0;

protected:
	std::deque<std::function<void(void)>> dispatch_queue_;
};

#endif /* _SCHEDULER_H_ */
