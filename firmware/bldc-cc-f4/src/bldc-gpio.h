#ifndef _BLDC_GPIOS_H_
#define _BLDC_GPIOS_H_

#include "bldc-include.h"

#ifdef __cplusplus
extern "C" {
#endif

void bldc_gpio_config();
void bldc_gpio_write(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t value);
void bldc_gpio_activelow_write(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t value, uint32_t activelow);
void bldc_gpio_toggle(GPIO_TypeDef *GPIOx, uint32_t PinMask);
uint32_t bldc_gpio_read(GPIO_TypeDef *GPIOx, uint32_t PinMask);
uint32_t bldc_gpio_activelow_read(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t activelow);

#ifdef __cplusplus
}
#endif

#endif
