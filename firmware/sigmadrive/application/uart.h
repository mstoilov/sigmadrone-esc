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
#include "cmsis_os2.h"
#include "stm32f745xx.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_uart.h"
#include "stm32f7xx_hal.h"

#include "ring.h"

class Uart {
public:
	using handle_map_type = std::map<UART_HandleTypeDef*, Uart*>;
	Uart(size_t poll_delay = 5);
	virtual ~Uart();
	void Attach(UART_HandleTypeDef* huart);
	void Detach();
	size_t TransmitOnce(const char* buffer, size_t nsize);
	size_t Transmit(const char* buffer, size_t nsize);
	size_t Transmit(const std::string& str);

	void TransmitCompleteCallback();
	size_t Receive(char* buffer, size_t nsize);
	std::string GetLine();
	void ReceiveCompleteCallback();
	void ErrorCallback();

	static handle_map_type handle_map_;

	static const uint32_t EVENT_FLAG_DATA = (1u << 9);

protected:
	size_t ReceiveOnce(char* buffer, size_t nsize);

public:
	UART_HandleTypeDef* huart_ = nullptr;

protected:
	volatile bool transmitting_ = false;
	Ring<char, 1024> tx_ringbuf_;
	Ring<char, 2048> rx_ringbuf_;
	size_t poll_delay_;
	osEventFlagsId_t event_;
};

#endif /* APPLICATION_UART_H_ */
