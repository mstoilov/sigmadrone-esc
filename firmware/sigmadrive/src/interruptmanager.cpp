/*
 * interruptmanager.cpp
 *
 *  Created on: Dec 9, 2018
 *      Author: mstoilov
 */

#include <iostream>
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
	InterruptManager::instance().vectors_[vector]();
}

extern "C"
void VectorHandlerC()
{
	InterruptManageVectorHandler();
}

InterruptManager::InterruptManager()
{
	relocate_interrupt_table();
	for (size_t i = 0; i < vectors_.size(); i++)
		vectors_[i] = [](void){};
}

void InterruptManager::EnableIRQ(IRQn_Type irq, uint32_t priority)
{
	NVIC_SetPriority(irq, priority);
	NVIC_EnableIRQ(irq);
}

void InterruptManager::DisableIRQ(IRQn_Type irq)
{
	NVIC_DisableIRQ(irq);
}


void InterruptManager::Callback(IRQn_Type irq, const std::function<void(void)>& callback)
{
	vectors_[irq + 16] = callback;
	volatile unsigned int* newtable = &__relocated_vectors;
	newtable[irq + 16] = (uint32_t)VectorHandlerC;
}

void InterruptManager::VectorHandler(IRQn_Type irq, void (*VectorHandlerC)(void))
{
	volatile unsigned int* newtable = &__relocated_vectors;
	newtable[irq + 16] = (uint32_t)VectorHandlerC;
}

