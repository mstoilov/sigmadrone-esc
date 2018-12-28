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

inline void InterruptManageVectorHandler()
{
	uint32_t vector = __get_xPSR() & 0xFF;
	InterruptManager& im = InterruptManager::instance();
	im.vectors_[vector]();
}

extern "C"
void VectorHandlerC()
{
	InterruptManageVectorHandler();
}

void InterruptManager::DebugBrakePoint()
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
		vectors_[i] = [](void){DebugBrakePoint();};
}

void InterruptManager::Callback(unsigned int irq, const std::function<void(void)>& callback)
{
	vectors_[irq + 16] = callback;
	volatile unsigned int* newtable = &__relocated_vectors;
	newtable[irq + 16] = (uint32_t)VectorHandlerC;
}
