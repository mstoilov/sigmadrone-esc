
#ifndef _INTERRUPTMANAGER_H_
#define _INTERRUPTMANAGER_H_

#include <array>
#include <functional>

class InterruptManager {
public:
	virtual ~InterruptManager();
	static InterruptManager& instance()
	{
		static InterruptManager object;

		return object;
	}

	void subscribe(unsigned int irq, const std::function<void(void)>& callback);

	template<typename T>
	void subscribe(unsigned int irq, void (T::*callback)(void), T* object)
	{
		subscribe(irq, [=](void){(object->*callback)();});
	}


private:
	InterruptManager();
	static void debug_brake_point();
	friend void interrupt_manager_vector_handler();

private:
	std::array<std::function<void(void)>, 16 + SPI5_IRQn> vectors_;
};

#endif /* _INTERRUPTMANAGER_H_ */
