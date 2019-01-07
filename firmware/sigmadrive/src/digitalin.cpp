#include "digitalin.h"
#include "interruptmanager.h"

static DigitalIn* g_interrupt[16] = {
		nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
		nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, };


void DigitalIn::IrqHandler(size_t line)
{
	/* Manage Flags */
	uint32_t exti_line = 1 << line;
	if (LL_EXTI_IsActiveFlag_0_31(exti_line) != RESET) {
		LL_EXTI_ClearFlag_0_31(exti_line);

		if (g_interrupt[line])
			g_interrupt[line]->callback_();
	}
}

void DigitalIn::IrqHandlers(size_t begin, size_t size)
{
	for (size_t i = begin; i < size; i++)
		IrqHandler(i);
}


DigitalIn::~DigitalIn()
{
	uint32_t exti_line = 1 << LL_PINNUM(pin_);
	if (LL_SYSCFG_GetEXTISource(exti_line) == LL_PORTNUM(pin_)) {
		g_interrupt[LL_PINNUM(pin_)] = nullptr;
	}
}

void DigitalIn::EXTILineEnable(uint32_t portnum, uint32_t linenum, uint32_t irq_priority)
{
	__SYSCFG_CLK_ENABLE();

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	if (linenum < 16) {
		uint32_t exti_line = 0;
		g_interrupt[linenum] = this;
		switch (linenum) {
		case 0:
			NVIC_EnableIRQ(EXTI0_IRQn);
			NVIC_SetPriority(EXTI0_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE0;
			InterruptManager::instance().Callback(EXTI0_IRQn, [](void){IrqHandler(0);});
			break;
		case 1:
			NVIC_EnableIRQ(EXTI1_IRQn);
			NVIC_SetPriority(EXTI1_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE1;
			InterruptManager::instance().Callback(EXTI1_IRQn, [](void){IrqHandler(1);});
			break;
		case 2:
			NVIC_EnableIRQ(EXTI2_IRQn);
			NVIC_SetPriority(EXTI2_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE2;
			InterruptManager::instance().Callback(EXTI2_IRQn, [](void){IrqHandler(2);});
			break;
		case 3:
			NVIC_EnableIRQ(EXTI3_IRQn);
			NVIC_SetPriority(EXTI3_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE3;
			InterruptManager::instance().Callback(EXTI3_IRQn, [](void){IrqHandler(3);});
			break;
		case 4:
			NVIC_EnableIRQ(EXTI4_IRQn);
			NVIC_SetPriority(EXTI4_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE4;
			InterruptManager::instance().Callback(EXTI4_IRQn, [=](void){IrqHandler(4);});
			break;
		case 5:
			NVIC_EnableIRQ(EXTI9_5_IRQn);
			NVIC_SetPriority(EXTI9_5_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE5;
			InterruptManager::instance().Callback(EXTI9_5_IRQn, [](void){IrqHandlers(5,5);});
			break;
		case 6:
			NVIC_EnableIRQ(EXTI9_5_IRQn);
			NVIC_SetPriority(EXTI9_5_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE6;
			InterruptManager::instance().Callback(EXTI9_5_IRQn, [](void){IrqHandlers(5,5);});
			break;
		case 7:
			NVIC_EnableIRQ(EXTI9_5_IRQn);
			NVIC_SetPriority(EXTI9_5_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE7;
			InterruptManager::instance().Callback(EXTI9_5_IRQn, [](void){IrqHandlers(5,5);});
			break;
		case 8:
			NVIC_EnableIRQ(EXTI9_5_IRQn);
			NVIC_SetPriority(EXTI9_5_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE8;
			InterruptManager::instance().Callback(EXTI9_5_IRQn, [](void){IrqHandlers(5,5);});
			break;
		case 9:
			NVIC_EnableIRQ(EXTI9_5_IRQn);
			NVIC_SetPriority(EXTI9_5_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE9;
			InterruptManager::instance().Callback(EXTI9_5_IRQn, [](void){IrqHandlers(5,5);});
			break;
		case 10:
			NVIC_EnableIRQ(EXTI15_10_IRQn);
			NVIC_SetPriority(EXTI15_10_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE10;
			InterruptManager::instance().Callback(EXTI15_10_IRQn, [](void){IrqHandlers(10,5);});
			break;
		case 11:
			NVIC_EnableIRQ(EXTI15_10_IRQn);
			NVIC_SetPriority(EXTI15_10_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE11;
			InterruptManager::instance().Callback(EXTI15_10_IRQn, [](void){IrqHandlers(10,5);});
			break;
		case 12:
			NVIC_EnableIRQ(EXTI15_10_IRQn);
			NVIC_SetPriority(EXTI15_10_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE12;
			InterruptManager::instance().Callback(EXTI15_10_IRQn, [](void){IrqHandlers(10,5);});
			break;
		case 13:
			NVIC_EnableIRQ(EXTI15_10_IRQn);
			NVIC_SetPriority(EXTI15_10_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE13;
			InterruptManager::instance().Callback(EXTI15_10_IRQn, [](void){IrqHandlers(10,5);});
			break;
		case 14:
			NVIC_EnableIRQ(EXTI15_10_IRQn);
			NVIC_SetPriority(EXTI15_10_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE14;
			InterruptManager::instance().Callback(EXTI15_10_IRQn, [](void){IrqHandlers(10,5);});
			break;
		case 15:
			NVIC_EnableIRQ(EXTI15_10_IRQn);
			NVIC_SetPriority(EXTI15_10_IRQn, irq_priority);
			exti_line = LL_SYSCFG_EXTI_LINE15;
			InterruptManager::instance().Callback(EXTI15_10_IRQn, [](void){IrqHandlers(10,5);});
			break;
		default:
			exti_line = 0;
			break;
		}
		LL_SYSCFG_SetEXTISource(portnum, exti_line);
	}
}
