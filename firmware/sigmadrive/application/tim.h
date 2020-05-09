/*
 * tim.h
 *
 *  Created on: Sep 6, 2019
 *      Author: mstoilov
 */

#ifndef APPLICATION_TIM_H_
#define APPLICATION_TIM_H_

#include <map>
#include <functional>
#include "stm32f745xx.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_tim.h"


class Tim {
public:
    Tim();
    virtual ~Tim();
    void attach();

};

#endif /* APPLICATION_TIM_H_ */
