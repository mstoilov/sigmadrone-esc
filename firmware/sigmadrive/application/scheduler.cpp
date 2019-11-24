/*
 * scheduler.cpp
 *
 *  Created on: Nov 22, 2019
 *      Author: mstoilov
 */

//#include "main.h"
#include "scheduler.h"

Scheduler::Scheduler()
{
	// TODO Auto-generated constructor stub

}

Scheduler::~Scheduler()
{
	// TODO Auto-generated destructor stub
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
	if (!dispatch_queue_.empty()) {
		SignalThreadTask();
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
		if (WaitTask(50)) {
			while (!dispatch_queue_.empty()) {
				std::function<void(void)> task = [](void){};

				taskENTER_CRITICAL();
				if (!dispatch_queue_.empty()) {
					task = dispatch_queue_.front();
					dispatch_queue_.pop_front();
				}
				taskEXIT_CRITICAL();

				/*
				 * Clear ABORT in case it was set
				 */
				osThreadFlagsClear(THREAD_SIGNAL_ABORT);

				task();
			}
		}
	}
}

static void RunSchedulerLoopWrapper(const void* ctx)
{
	extern struct _reent *_impure_data_ptr;
	*_impure_ptr = *_impure_data_ptr;

	reinterpret_cast<Scheduler*>(const_cast<void*>(ctx))->RunSchedulerLoop();
	reinterpret_cast<Scheduler*>(const_cast<void*>(ctx))->scheduler_thread_id_ = 0;
	osThreadExit();
}

void Scheduler::StartDispatcherThread()
{
	osThreadDef(RunSchedulerLoopWrapper, osPriorityHigh, 0, 4000);
	scheduler_thread_id_ = osThreadCreate(&os_thread_def_RunSchedulerLoopWrapper, this);
}

bool Scheduler::WaitUpdate(uint32_t timeout_msec)
{
	return (osSignalWait(THREAD_SIGNAL_UPDATE, timeout_msec).status == osEventSignal) ? true : false;
}

bool Scheduler::WaitTask(uint32_t timeout)
{
	return (osSignalWait(THREAD_SIGNAL_TASK, timeout).status == osEventSignal) ? true : false;
}

bool Scheduler::WaitAbort(uint32_t timeout)
{
	return (osSignalWait(THREAD_SIGNAL_ABORT, timeout).status == osEventSignal) ? true : false;
}

void Scheduler::SignalThreadUpdate()
{
	if (scheduler_thread_id_)
		osSignalSet(scheduler_thread_id_, THREAD_SIGNAL_UPDATE);
}

void Scheduler::SignalThreadTask()
{
	if (scheduler_thread_id_)
		osSignalSet(scheduler_thread_id_, THREAD_SIGNAL_TASK);
}

void Scheduler::SignalThreadAbort()
{
	if (scheduler_thread_id_)
		osSignalSet(scheduler_thread_id_, THREAD_SIGNAL_ABORT);
}


