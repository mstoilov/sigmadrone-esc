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
	void attach(UART_HandleTypeDef* huart);
	void detach();
	ssize_t transmit(const char* buffer, size_t nsize);
	void transmit_complete();
	ssize_t receive(char* buffer, size_t nsize);
	void receive_complete();

	static handle_map_type handle_map_;

public:
	UART_HandleTypeDef* huart_ = nullptr;

protected:
	volatile bool transmitting_ = false;
	Ring<char, 200> tx_ringbuf_;
	Ring<char, 50> rx_ringbuf_;
};

#endif /* APPLICATION_UART_H_ */
