
#ifndef _INTERRUPTMANAGER_H_
#define _INTERRUPTMANAGER_H_

#include <array>
#include <vector>
#include <functional>


class InterruptManager {
public:
	virtual ~InterruptManager() = default;
	static InterruptManager& instance()
	{
		static InterruptManager object;

		return object;
	}

	void DisableIRQ(IRQn_Type irq);
	void EnableIRQ(IRQn_Type irq, uint32_t priority);
	void Callback_EnableIRQ(IRQn_Type irq, uint32_t priority, const std::function<void(void)>& callback)
	{
		Callback(irq, callback);
		EnableIRQ(irq, priority);
	}

	template<typename T>
	void Callback_EnableIRQ(IRQn_Type irq, uint32_t priority, void (T::*func)(void), T* object)
	{
		Callback(irq, func, object);
		EnableIRQ(irq, priority);
	}

	void Callback(IRQn_Type irq, const std::function<void(void)>& callback);

	template<typename T>
	void Callback(IRQn_Type irq, void (T::*func)(void), T* object)
	{
		Callback(irq, [=](void){(object->*func)();});
	}

	std::function<void(void)> GetIrqHandler(unsigned int irq)
	{
		return vectors_[16 + irq];
	}

private:
	InterruptManager();
	friend void InterruptManageVectorHandler();

private:
	std::array<std::function<void(void)>, 16 + SPI5_IRQn> vectors_;
};

#endif /* _INTERRUPTMANAGER_H_ */
