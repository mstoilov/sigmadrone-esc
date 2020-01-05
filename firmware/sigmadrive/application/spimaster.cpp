/*
 * spimaster.cpp
 *
 *  Created on: Sep 1, 2019
 *      Author: mstoilov
 */

#include <string.h>
#include <stdio.h>
#include "spimaster.h"

SPIMaster::SPIMaster() {
	// TODO Auto-generated constructor stub

}

SPIMaster::~SPIMaster() {
	// TODO Auto-generated destructor stub
}

void SPIMaster::Attach(SPI_HandleTypeDef* hspi)
{
	hspi_ = hspi;

//	LL_SPI_InitTypeDef spi_init;
//	memset(&spi_init, 0, sizeof(spi_init));
//
//
//	spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
//	spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
//	spi_init.ClockPolarity = LL_SPI_POLARITY_LOW;
//	spi_init.ClockPhase = LL_SPI_PHASE_2EDGE;
//	spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
//	spi_init.CRCPoly = 10;
//	spi_init.DataWidth = LL_SPI_DATAWIDTH_16BIT;
//	spi_init.BitOrder = LL_SPI_MSB_FIRST;
//	spi_init.NSS = LL_SPI_NSS_SOFT;
//	spi_init.Mode = LL_SPI_MODE_MASTER;
//
//	ErrorStatus error = LL_SPI_Init(SPI3, &spi_init);
//	if (error != SUCCESS) {
//		printf("SPI init failed\n");
//	}
}

void SPIMaster::Detach()
{

}

void SPIMaster::ChipSelect(GPIO_TypeDef* NSS_GPIOx, uint16_t NSS_GPIO_Pin, bool select)
{
	HAL_Delay(5);
	HAL_GPIO_WritePin(NSS_GPIOx, NSS_GPIO_Pin, select ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_Delay(5);
}

bool SPIMaster::Transmit(uint8_t *pData, uint16_t Size)
{
	return (HAL_SPI_Transmit(hspi_, pData, Size, timeout) == HAL_OK) ? true : false;
}

bool SPIMaster::Receive(uint8_t *pData, uint16_t Size)
{
	return (HAL_SPI_Receive(hspi_, pData, Size, timeout) == HAL_OK) ? true : false;
}

bool SPIMaster::TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	return (HAL_SPI_TransmitReceive(hspi_, pTxData, pRxData, Size, timeout) == HAL_OK) ? true : false;
}

