/*
 * uart.h
 *
 *  Created on: Aug 30, 2019
 *      Author: mstoilov
 */

#ifndef APPLICATION_UART_H_
#define APPLICATION_UART_H_

#include <array>
#include <map>
#include <functional>
#include "stm32f745xx.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_uart.h"

#include "ring.h"

class Uart {
public:
	using handle_map_type = std::map<UART_HandleTypeDef*, Uart*>;
	Uart();
	virtual ~Uart();
	void Attach(UART_HandleTypeDef* huart);
	void Detach();
	ssize_t Transmit(const char* buffer, size_t nsize);
	void TransmitCompleteCallback();
	ssize_t Receive(char* buffer, size_t nsize);
	void ReceiveCompleteCallback();

	static handle_map_type handle_map_;

public:
	UART_HandleTypeDef* huart_ = nullptr;

protected:
	volatile bool transmitting_ = false;
	Ring<char, 1024> tx_ringbuf_;
	Ring<char, 1024> rx_ringbuf_;
};

#endif /* APPLICATION_UART_H_ */
