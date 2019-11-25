/*
 * scheduler.cpp
 *
 *  Created on: Nov 22, 2019
 *      Author: mstoilov
 */

//#include "main.h"
#include "scheduler.h"

#if (osCMSIS < 0x20000U)
extern "C"
uint32_t osThreadFlagsClear(uint32_t flags)
{
	TaskHandle_t hTask;
	uint32_t rflags, cflags;

	hTask = xTaskGetCurrentTaskHandle();

	if (xTaskNotifyAndQuery(hTask, 0, eNoAction, &cflags) == ((BaseType_t) 1)) {
		rflags = cflags;
		cflags &= ~flags;

		if (xTaskNotify(hTask, cflags, eSetValueWithOverwrite) != ((BaseType_t) 1)) {
			rflags = (uint32_t) -1;
		}
	} else {
		rflags = (uint32_t) -1;
	}

	/* Return flags before clearing */
	return (rflags);
}
#endif

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
		if (WaitTask(50)) {
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

static void RunSchedulerLoopWrapper(const void* ctx)
{
	extern struct _reent *_impure_data_ptr;
	*_impure_ptr = *_impure_data_ptr;

	reinterpret_cast<Scheduler*>(const_cast<void*>(ctx))->RunSchedulerLoop();
	reinterpret_cast<Scheduler*>(const_cast<void*>(ctx))->scheduler_thread_id_ = 0;
#if (osCMSIS >= 0x20000U)
	osThreadExit();
#else
	while(1){}
#endif
}

void Scheduler::StartDispatcherThread()
{
#if (osCMSIS >= 0x20000U)
	osThreadDef(RunSchedulerLoopWrapper, osPriorityHigh, 0, 4000);
	scheduler_thread_id_ = osThreadCreate(&os_thread_def_RunSchedulerLoopWrapper, this);
#else
	osThreadDef(RunSchedulerLoopDef, RunSchedulerLoopWrapper, osPriorityHigh, 0, 4000);
	scheduler_thread_id_ = osThreadCreate(osThread(RunSchedulerLoopDef), this);
#endif
}

uint32_t Scheduler::WaitSignalsPriv(uint32_t s, uint32_t timeout_msec)
{
	uint32_t t0, td, tout = timeout_msec;
	uint32_t ret = 0;

	do {
		t0 = xTaskGetTickCount();
		osEvent e = osSignalWait(0, tout);
		ret = (e.status == osEventSignal && (e.value.signals & s)) ? e.value.signals & s : 0;

		/* Update timeout */
		td = xTaskGetTickCount() - t0;
		tout = (td > tout) ? 0 : tout - td;
	} while (!ret && tout);

	return ret;
}

uint32_t Scheduler::WaitSignals(uint32_t s, uint32_t timeout_msec)
{
	return WaitSignalsPriv(s, timeout_msec);
}

bool Scheduler::WaitUpdate(uint32_t timeout_msec)
{
	return WaitSignalsPriv(THREAD_SIGNAL_UPDATE, timeout_msec);
}

bool Scheduler::WaitTask(uint32_t timeout_msec)
{
	return WaitSignalsPriv(THREAD_SIGNAL_TASK, timeout_msec);
}

bool Scheduler::WaitAbort(uint32_t timeout_msec)
{
	return WaitSignalsPriv(THREAD_SIGNAL_ABORT, timeout_msec);
}

bool Scheduler::WaitIdle(uint32_t timeout_msec)
{
	return WaitSignalsPriv(THREAD_SIGNAL_IDLE, timeout_msec);
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

