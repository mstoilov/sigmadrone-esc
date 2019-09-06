/*
 * spimaster.h
 *
 *  Created on: Sep 1, 2019
 *      Author: mstoilov
 */

#ifndef APPLICATION_SPIMASTER_H_
#define APPLICATION_SPIMASTER_H_

#include "stm32f745xx.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_spi.h"

#include "stm32f7xx_ll_spi.h"

class SPIMaster {
public:
	static const uint32_t timeout = 100;

	SPIMaster();
	virtual ~SPIMaster();

	void attach(SPI_HandleTypeDef* hspi);
	void detach();
	void chip_select(GPIO_TypeDef* NSS_GPIOx, uint16_t NSS_GPIO_Pin, bool select);

	bool transmit(uint8_t *pData, uint16_t Size);
	bool receive(uint8_t *pData, uint16_t Size);
	bool transmit_receive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

public:
	SPI_HandleTypeDef* hspi_;
};

#endif /* APPLICATION_SPIMASTER_H_ */
