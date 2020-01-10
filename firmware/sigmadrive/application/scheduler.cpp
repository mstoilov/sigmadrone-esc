/*
 * scheduler.cpp
 *
 *  Created on: Nov 22, 2019
 *      Author: mstoilov
 */

#include <string.h>

#include "stm32f7xx.h"
#include "stm32f7xx_hal_gpio.h"
#include "main.h"

#include "scheduler.h"

Scheduler::Scheduler()
	: idle_task_([](){})
	, abort_task_([](){})
	, event_dispatcher_idle_(osEventFlagsNew(NULL))

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

void Scheduler::SetAbortTask(const std::function<void(void)>& task)
{
	abort_task_ = task;
}

void Scheduler::OnUpdate()
{
	if (update_handler_) {
		if (!(*update_handler_)() || dispatch_queue_.empty()) {
			update_handler_ = nullptr;
			SignalThreadUpdateDone();
		}
	}
}

bool Scheduler::RunUpdateHandler(const std::function<bool(void)>& update_handler)
{
	if (dispatch_queue_.empty())
		return false;
	update_handler_ = &update_handler;
	while (!WaitSignalUpdateDone(-1))
		;
	return true;
}

void Scheduler::Abort()
{
	__disable_irq();
	dispatch_queue_.clear();
	__enable_irq();
	SignalThreadAbort();
	abort_task_();
}

void Scheduler::Run()
{
	if (!dispatch_queue_.empty()) {
		dispatching_ = true;
		SignalThreadTask();
	}
}

void Scheduler::RunWaitForCompletion()
{
	if (!dispatch_queue_.empty()) {
		/*
		 * Clear IDLE in case it was set
		 */
		osEventFlagsClear(event_dispatcher_idle_, EVENT_FLAG_IDLE);
		dispatching_ = true;
		SignalThreadTask();
		while (!WaitEventIdle(osWaitForever))
			;
	}
}

bool Scheduler::IsDispatching()
{
	return dispatching_;
}

void Scheduler::AddTask(const std::function<void(void)>& task)
{
	if (dispatching_)
		throw std::runtime_error("New tasks can't be added while the scheduler is dispatching tasks.");
	taskENTER_CRITICAL();
	dispatch_queue_.push_back(task);
	taskEXIT_CRITICAL();
}

void Scheduler::RunSchedulerLoop()
{
	for (;;) {
		if (WaitSignalTask(run_idle_ms_)) {
			/*
			 * Clear IDLE in case it was set
			 */
			osEventFlagsClear(event_dispatcher_idle_, EVENT_FLAG_IDLE);

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
				osThreadFlagsClear(THREAD_FLAG_ABORT);

				/*
				 * Turn on Status LED
				 */
				HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

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
			dispatching_ = false;
		} else {
			HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
			EventThreadIdle();
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
		t0 = osKernelGetTickCount();
		ret = osThreadFlagsWait(s, osFlagsWaitAny, tout);
		if (ret & osFlagsError)
			ret = 0;
		ret &= s;
		/* Update timeout */
		td = osKernelGetTickCount() - t0;
		tout = (td > tout) ? 0 : tout - td;
	} while (!ret && tout);
	return (ret & s);
}

uint32_t Scheduler::WaitEvents(osEventFlagsId_t event, uint32_t s, uint32_t timeout_msec)
{
	uint32_t t0, td, tout = timeout_msec;
	uint32_t ret = 0;

	do {
		t0 = osKernelGetTickCount();
		ret = osEventFlagsWait(event, s, osFlagsWaitAny, tout);
		if (ret & osFlagsError)
			ret = 0;
		ret &= s;
		/* Update timeout */
		td = osKernelGetTickCount() - t0;
		tout = (td > tout) ? 0 : tout - td;
	} while (!ret && tout);
	return (ret & s);
}


bool Scheduler::WaitSignalUpdateDone(uint32_t timeout_msec)
{
	return (WaitSignals(THREAD_FLAG_UPDATE_DONE, timeout_msec) == THREAD_FLAG_UPDATE_DONE) ? true : false;
}


bool Scheduler::WaitSignalTask(uint32_t timeout_msec)
{
	return (WaitSignals(THREAD_FLAG_TASK, timeout_msec) == THREAD_FLAG_TASK) ? true : false;
}

bool Scheduler::WaitSignalAbort(uint32_t timeout_msec)
{
	return (WaitSignals(THREAD_FLAG_ABORT, timeout_msec) == THREAD_FLAG_ABORT) ? true : false;
}

bool Scheduler::WaitEventIdle(uint32_t timeout_msec)
{
	return (WaitEvents(event_dispatcher_idle_, EVENT_FLAG_IDLE, timeout_msec) == EVENT_FLAG_IDLE) ? true : false;
}

void Scheduler::SignalThreadUpdateDone()
{
	if (scheduler_thread_id_) {
		osThreadFlagsSet(scheduler_thread_id_, THREAD_FLAG_UPDATE_DONE);
	}
}

void Scheduler::SignalThreadTask()
{
	if (scheduler_thread_id_)
		osThreadFlagsSet(scheduler_thread_id_, THREAD_FLAG_TASK);
}

void Scheduler::SignalThreadAbort()
{
	if (scheduler_thread_id_)
		osThreadFlagsSet(scheduler_thread_id_, THREAD_FLAG_ABORT);
}

void Scheduler::EventThreadIdle()
{
	osEventFlagsSet(event_dispatcher_idle_, EVENT_FLAG_IDLE);
}

