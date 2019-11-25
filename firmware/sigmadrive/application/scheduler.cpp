/*
 * scheduler.cpp
 *
 *  Created on: Nov 22, 2019
 *      Author: mstoilov
 */

//#include "main.h"
#include <string.h>
#include "scheduler.h"

Scheduler::Scheduler()
	: idle_task_([](){})
{
	// TODO Auto-generated constructor stub

}

Scheduler::~Scheduler()
{
	// TODO Auto-generated destructor stub
}

void Scheduler::SetIdleTask(const std::function<void(void)>& task)
{
	idle_task_ = task;
}

void Scheduler::Abort()
{
	taskENTER_CRITICAL();
	dispatch_queue_.clear();
	taskEXIT_CRITICAL();

	SignalThreadAbort();

}

void Scheduler::Run()
{
	/*
	 * TBD: Check the dispatcher is not
	 * already running.
	 */
	if (!dispatch_queue_.empty()) {
		SignalThreadTask();
	}
}

void Scheduler::RunWaitForCompletion()
{
	/*
	 * TBD: Check the dispatcher is not
	 * already running.
	 */
	if (!dispatch_queue_.empty()) {
		blocked_thread_id_ = osThreadGetId();

		/*
		 * Clear IDLE in case it was set
		 */
		osThreadFlagsClear(THREAD_SIGNAL_IDLE);

		SignalThreadTask();

		while (!WaitIdle(osWaitForever))
			;
	}
}

void Scheduler::AddTask(const std::function<void(void)>& task)
{
	taskENTER_CRITICAL();
	dispatch_queue_.push_back(task);
	taskEXIT_CRITICAL();

}

void Scheduler::RunSchedulerLoop()
{
	for (;;) {
		if (WaitTask(5000)) {
			/*
			 * Clear UPDATE signal
			 */
			osThreadFlagsClear(THREAD_SIGNAL_UPDATE);


			while (!dispatch_queue_.empty()) {
				std::function<void(void)> task = [](void){};

				/*
				 * Init the task
				 */
				taskENTER_CRITICAL();
				if (!dispatch_queue_.empty())
					task = dispatch_queue_.front();
				taskEXIT_CRITICAL();

				/*
				 * Clear ABORT in case it was set
				 */
				osThreadFlagsClear(THREAD_SIGNAL_ABORT);

				/*
				 * Run the task
				 */
				task();

				/*
				 * Remove the task from the queue
				 */
				taskENTER_CRITICAL();
				if (!dispatch_queue_.empty())
					dispatch_queue_.pop_front();
				taskEXIT_CRITICAL();

			}

			SignalThreadIdle();

		} else {
			idle_task_();
		}
	}

}

static void RunSchedulerLoopWrapper(void* ctx)
{
	extern struct _reent *_impure_data_ptr;
	*_impure_ptr = *_impure_data_ptr;

	reinterpret_cast<Scheduler*>(const_cast<void*>(ctx))->RunSchedulerLoop();
	reinterpret_cast<Scheduler*>(const_cast<void*>(ctx))->scheduler_thread_id_ = 0;
	osThreadExit();
}

void Scheduler::StartDispatcherThread()
{
	osThreadAttr_t task_attributes;
	memset(&task_attributes, 0, sizeof(osThreadAttr_t));
	task_attributes.name = "RunSchedulerLoopTask";
	task_attributes.priority = (osPriority_t) osPriorityHigh;
	task_attributes.stack_size = 4000;
	scheduler_thread_id_ = osThreadNew(RunSchedulerLoopWrapper, this, &task_attributes);
}

uint32_t Scheduler::WaitSignals(uint32_t s, uint32_t timeout_msec)
{
	uint32_t t0, td, tout = timeout_msec;
	uint32_t ret = 0;

	do {
		t0 = xTaskGetTickCount();
		ret = osThreadFlagsWait(s, osFlagsWaitAny, tout);
		if (ret & osFlagsError)
			ret = 0;
		ret &= s;
		/* Update timeout */
		td = xTaskGetTickCount() - t0;
		tout = (td > tout) ? 0 : tout - td;
	} while (!ret && tout);
	return (ret & s);
}

bool Scheduler::WaitUpdate(uint32_t timeout_msec)
{
	return (WaitSignals(THREAD_SIGNAL_UPDATE, timeout_msec) == THREAD_SIGNAL_UPDATE) ? true : false;
}

bool Scheduler::WaitTask(uint32_t timeout_msec)
{
	return (WaitSignals(THREAD_SIGNAL_TASK, timeout_msec) == THREAD_SIGNAL_TASK) ? true : false;
}

bool Scheduler::WaitAbort(uint32_t timeout_msec)
{
	return (WaitSignals(THREAD_SIGNAL_ABORT, timeout_msec) == THREAD_SIGNAL_ABORT) ? true : false;
}

bool Scheduler::WaitIdle(uint32_t timeout_msec)
{
	return (WaitSignals(THREAD_SIGNAL_IDLE, timeout_msec) == THREAD_SIGNAL_IDLE) ? true : false;
}

void Scheduler::SignalThreadUpdate()
{
	if (scheduler_thread_id_ && !dispatch_queue_.empty())
		osThreadFlagsSet(scheduler_thread_id_, THREAD_SIGNAL_UPDATE);
}

void Scheduler::SignalThreadTask()
{
	if (scheduler_thread_id_)
		osThreadFlagsSet(scheduler_thread_id_, THREAD_SIGNAL_TASK);
}

void Scheduler::SignalThreadAbort()
{
	if (scheduler_thread_id_)
		osThreadFlagsSet(scheduler_thread_id_, THREAD_SIGNAL_ABORT);
}

void Scheduler::SignalThreadIdle()
{
	if (blocked_thread_id_)
		osThreadFlagsSet(blocked_thread_id_, THREAD_SIGNAL_IDLE);
}

