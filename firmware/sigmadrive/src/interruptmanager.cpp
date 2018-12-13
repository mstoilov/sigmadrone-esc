/*
 * interruptmanager.cpp
 *
 *  Created on: Dec 9, 2018
 *      Author: mstoilov
 */

#include <cmsis_device.h>
#include "cortexm/ExceptionHandlers.h"
#include "interruptmanager.h"

extern unsigned int __relocated_vectors;

void relocate_interrupt_table()
{
	volatile unsigned int* newtable = &__relocated_vectors;
	volatile unsigned int* oldtable = 0;

	for (size_t i = 0; i < 256; i++) {
		newtable[i] = oldtable[i];
	}

	__DMB();
	SCB->VTOR = ((unsigned int)newtable);
	__DSB();
}

inline void interrupt_manager_vector_handler()
{
	uint32_t vector = __get_xPSR() & 0xFF;
	InterruptManager::instance().vectors_[vector]();
}

extern "C"
void vector_handler()
{
	interrupt_manager_vector_handler();
}

void InterruptManager::debug_brake_point()
{
#if defined(DEBUG)
	__DEBUG_BKPT();
#endif
	while (1)
	{
	}
}

InterruptManager::InterruptManager()
{
	relocate_interrupt_table();
	for (size_t i = 0; i < vectors_.size(); i++)
		vectors_[i] = [](void){debug_brake_point();};
}

InterruptManager::~InterruptManager()
{
	// TODO Auto-generated destructor stub
}

void InterruptManager::callback(unsigned int irq, const std::function<void(void)>& callback)
{
	vectors_[irq + 16] = callback;
	volatile unsigned int* newtable = &__relocated_vectors;
	newtable[irq + 16] = (uint32_t)vector_handler;
}

