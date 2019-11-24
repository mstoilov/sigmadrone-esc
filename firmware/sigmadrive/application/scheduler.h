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
	void RunWaitForCompletion();
	void SetIdleTask(const std::function<void(void)>& task);
	void AddTask(const std::function<void(void)>& task);
	void RunSchedulerLoop();
	void SignalThreadUpdate();
	void SignalThreadTask();
	void SignalThreadAbort();
	void SignalThreadIdle();

	bool WaitUpdate(uint32_t timeout_msec);
	bool WaitTask(uint32_t timeout_msec);
	bool WaitAbort(uint32_t timeout_msec);
	bool WaitIdle(uint32_t timeout_msec);

	static const uint32_t THREAD_SIGNAL_TASK = (1u << 0);
	static const uint32_t THREAD_SIGNAL_UPDATE = (1u << 1);
	static const uint32_t THREAD_SIGNAL_ABORT = (1u << 2);
	static const uint32_t THREAD_SIGNAL_IDLE = (1u << 3);

	uint32_t WaitSignals(uint32_t s, uint32_t timeout_msec);

	uint32_t wait_timeout_ = (uint32_t)-1;
	osThreadId scheduler_thread_id_ = 0;
	osThreadId blocked_thread_id_ = 0;

protected:
	std::function<void(void)> idle_task_;
	std::deque<std::function<void(void)>> dispatch_queue_;
};

#endif /* _SCHEDULER_H_ */
