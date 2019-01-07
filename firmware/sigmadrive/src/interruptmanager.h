
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

	void Callback(unsigned int irq, const std::function<void(void)>& callback);

	template<typename T>
	void Callback(unsigned int irq, void (T::*func)(void), T* object)
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
