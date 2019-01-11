#include "digitalin.h"
#include "interruptmanager.h"

void DigitalIn::IrqHandler(size_t line)
{
	/* Manage Flags */
	uint32_t exti_line = 1 << line;
	if (LL_EXTI_IsActiveFlag_0_31(exti_line) != RESET) {
		LL_EXTI_ClearFlag_0_31(exti_line);
		callback_();
	}
}

DigitalIn::~DigitalIn()
{
}

void DigitalIn::EXTILineEnable(uint32_t portnum, uint32_t linenum, uint32_t irq_priority)
{
	__SYSCFG_CLK_ENABLE();

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	if (linenum < 16) {
		uint32_t exti_line = 0;
		switch (linenum) {
		case 0:
			exti_line = LL_SYSCFG_EXTI_LINE0;
			InterruptManager::instance().Callback_EnableIRQ(EXTI0_IRQn, irq_priority, [=](void){IrqHandler(0);});
			break;
		case 1:
			exti_line = LL_SYSCFG_EXTI_LINE1;
			InterruptManager::instance().Callback_EnableIRQ(EXTI1_IRQn, irq_priority, [=](void){IrqHandler(1);});
			break;
		case 2:
			exti_line = LL_SYSCFG_EXTI_LINE2;
			InterruptManager::instance().Callback_EnableIRQ(EXTI2_IRQn, irq_priority, [=](void){IrqHandler(2);});
			break;
		case 3:
			exti_line = LL_SYSCFG_EXTI_LINE3;
			InterruptManager::instance().Callback_EnableIRQ(EXTI3_IRQn, irq_priority, [=](void){IrqHandler(3);});
			break;
		case 4:
			exti_line = LL_SYSCFG_EXTI_LINE4;
			InterruptManager::instance().Callback_EnableIRQ(EXTI4_IRQn, irq_priority, [=](void){IrqHandler(4);});
			break;
		case 5:
			{
				auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(EXTI9_5_IRQn);
				exti_line = LL_SYSCFG_EXTI_LINE5;
				InterruptManager::instance().Callback_EnableIRQ(EXTI9_5_IRQn, irq_priority, [=](void){IrqHandler(5); OldIrqHandler();});
			}
			break;
		case 6:
			{
				auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(EXTI9_5_IRQn);
				exti_line = LL_SYSCFG_EXTI_LINE6;
				InterruptManager::instance().Callback_EnableIRQ(EXTI9_5_IRQn, irq_priority, [=](void){IrqHandler(6); OldIrqHandler();});
			}
			break;
		case 7:
			{
				auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(EXTI9_5_IRQn);
				exti_line = LL_SYSCFG_EXTI_LINE7;
				InterruptManager::instance().Callback_EnableIRQ(EXTI9_5_IRQn, irq_priority, [=](void){IrqHandler(7); OldIrqHandler();});
			}
			break;
		case 8:
			{
				auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(EXTI9_5_IRQn);
				exti_line = LL_SYSCFG_EXTI_LINE8;
				InterruptManager::instance().Callback_EnableIRQ(EXTI9_5_IRQn, irq_priority, [=](void){IrqHandler(8); OldIrqHandler();});
			}
			break;
		case 9:
			{
				auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(EXTI9_5_IRQn);
				exti_line = LL_SYSCFG_EXTI_LINE9;
				InterruptManager::instance().Callback_EnableIRQ(EXTI9_5_IRQn, irq_priority, [=](void){IrqHandler(9); OldIrqHandler();});
			}
			break;
		case 10:
			{
				auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(EXTI15_10_IRQn);
				exti_line = LL_SYSCFG_EXTI_LINE10;
				InterruptManager::instance().Callback_EnableIRQ(EXTI15_10_IRQn, irq_priority, [=](void){IrqHandler(10); OldIrqHandler();});
			}
			break;
		case 11:
			{
				auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(EXTI15_10_IRQn);
				exti_line = LL_SYSCFG_EXTI_LINE11;
				InterruptManager::instance().Callback_EnableIRQ(EXTI15_10_IRQn, irq_priority, [=](void){IrqHandler(11); OldIrqHandler();});
			}
			break;
		case 12:
			{
				auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(EXTI15_10_IRQn);
				exti_line = LL_SYSCFG_EXTI_LINE12;
				InterruptManager::instance().Callback_EnableIRQ(EXTI15_10_IRQn, irq_priority, [=](void){IrqHandler(12); OldIrqHandler();});
			}
			break;
		case 13:
			{
				auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(EXTI15_10_IRQn);
				exti_line = LL_SYSCFG_EXTI_LINE13;
				InterruptManager::instance().Callback_EnableIRQ(EXTI15_10_IRQn, irq_priority, [=](void){IrqHandler(13); OldIrqHandler();});
			}
			break;
		case 14:
			{
				auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(EXTI15_10_IRQn);
				exti_line = LL_SYSCFG_EXTI_LINE14;
				InterruptManager::instance().Callback_EnableIRQ(EXTI15_10_IRQn, irq_priority, [=](void){IrqHandler(14); OldIrqHandler();});
			}
			break;
		case 15:
			{
				auto OldIrqHandler = InterruptManager::instance().GetIrqHandler(EXTI15_10_IRQn);
				exti_line = LL_SYSCFG_EXTI_LINE15;
				InterruptManager::instance().Callback_EnableIRQ(EXTI15_10_IRQn, irq_priority, [=](void){IrqHandler(15); OldIrqHandler();});
			}
			break;
		default:
			exti_line = 0;
			break;
		}
		LL_SYSCFG_SetEXTISource(portnum, exti_line);
	}
}
