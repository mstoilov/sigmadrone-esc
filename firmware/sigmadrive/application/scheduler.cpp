/*
 * scheduler.cpp
 *
 *  Created on: Nov 22, 2019
 *      Author: mstoilov
 */

//#include "main.h"
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
		 * Clear ABORT in case it was set
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
		if (WaitTask(50)) {
			/*
			 * Clear UPDATE signal
			 */
			osThreadFlagsClear(THREAD_SIGNAL_UPDATE);

			while (!dispatch_queue_.empty()) {
				std::function<void(void)> task = [](void){};

				taskENTER_CRITICAL();
				if (!dispatch_queue_.empty())
					task = dispatch_queue_.front();
				taskEXIT_CRITICAL();

				/*
				 * Clear ABORT in case it was set
				 */
				osThreadFlagsClear(THREAD_SIGNAL_ABORT);
				task();

				taskENTER_CRITICAL();
				if (!dispatch_queue_.empty())
					dispatch_queue_.pop_front();
				taskEXIT_CRITICAL();

			}
		} else {
			SignalThreadIdle();
			idle_task_();
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

bool Scheduler::WaitForSignal(Signals s, uint32_t timeout_msec)
{
	uint32_t t0, td, tout = timeout_msec;
	t0 = osKernelGetTickCount();
	bool ret = false;

	do {
		osEvent e = osSignalWait(s, timeout_msec);

		ret = (e.status == osEventSignal && (e.value.signals & s)) ? true : false;

		/* Update timeout */
		td = xTaskGetTickCount() - t0;

		if (td > tout) {
			tout = 0;
		} else {
			tout -= td;
		}
	} while (!ret && tout);

	return ret;
}

bool Scheduler::WaitUpdate(uint32_t timeout_msec)
{
//	return (osSignalWait(THREAD_SIGNAL_UPDATE, timeout_msec).status == osEventSignal) ? true : false;

	return WaitForSignal(THREAD_SIGNAL_UPDATE, timeout_msec);

}

bool Scheduler::WaitTask(uint32_t timeout_msec)
{
//	return (osSignalWait(THREAD_SIGNAL_TASK, timeout).status == osEventSignal) ? true : false;

	return WaitForSignal(THREAD_SIGNAL_TASK, timeout_msec);
}

bool Scheduler::WaitAbort(uint32_t timeout_msec)
{
//	return (osSignalWait(THREAD_SIGNAL_ABORT, timeout_msec).status == osEventSignal) ? true : false;

	return WaitForSignal(THREAD_SIGNAL_ABORT, timeout_msec);

}

bool Scheduler::WaitIdle(uint32_t timeout_msec)
{
//	return (osSignalWait(THREAD_SIGNAL_IDLE, timeout_msec).status == osEventSignal) ? true : false;

	return WaitForSignal(THREAD_SIGNAL_IDLE, timeout_msec);

}

void Scheduler::SignalThreadUpdate()
{
	if (scheduler_thread_id_ && !dispatch_queue_.empty())
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

void Scheduler::SignalThreadIdle()
{
	if (blocked_thread_id_)
		osSignalSet(blocked_thread_id_, THREAD_SIGNAL_IDLE);
}

