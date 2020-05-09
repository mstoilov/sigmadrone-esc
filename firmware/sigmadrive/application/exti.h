/*
 * Exti.h
 *
 *  Created on: Sep 4, 2019
 *      Author: mstoilov
 */

#ifndef APPLICATION_EXTI_H_
#define APPLICATION_EXTI_H_

#include <stdint.h>
#include <map>
#include <functional>


class Exti {
public:
    using exti_map_type = std::map<uint16_t, Exti*>;
    Exti(uint16_t gpio_pin, const std::function<void(void)>& callback);
    virtual ~Exti();
    void SetCallback(const std::function<void(void)>& callback);
    static void GpioExtiCallback(uint16_t line);

    std::function<void(void)> callback_;
    static exti_map_type map_;

protected:
    uint16_t gpio_pin_;
};


#endif /* APPLICATION_EXTI_H_ */
