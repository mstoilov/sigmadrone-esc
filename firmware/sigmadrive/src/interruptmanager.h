
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


private:
	InterruptManager();
	static void DebugBrakePoint();
	friend void InterruptManageVectorHandler();

private:
#if USE_MULTICALLBACKS
	std::array<std::vector<std::function<void(void)>>, 16 + SPI5_IRQn> vectors_;
#else
	std::array<std::function<void(void)>, 16 + SPI5_IRQn> vectors_;
#endif
};

#endif /* _INTERRUPTMANAGER_H_ */
