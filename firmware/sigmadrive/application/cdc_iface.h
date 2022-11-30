/*
 * usb_cdc.h
 *
 *  Created on: Sep 16, 2019
 *      Author: mstoilov
 */

#ifndef _CDC_IFACE_H_
#define _CDC_IFACE_H_

#include <string>
#include <map>
#include "cmsis_os2.h"
#include "ring.h"
#include "usbd_cdc_if.h"


class CdcIface {
public:
	using handle_map_type = std::map<USBD_HandleTypeDef*, CdcIface*>;

	CdcIface();
	virtual ~CdcIface();
	void Attach(USBD_HandleTypeDef* usbd, bool start_tx_thread = false);
	int8_t ReceiveComplete(uint8_t* buf, uint32_t len);
	size_t Transmit(const char* buffer, size_t nsize);
	size_t Transmit(const std::string& str);
	size_t Receive(char* buffer, size_t nsize);
	size_t ReceiveLine(char* buffer, size_t nsize);
	void RunTxLoop();
	static handle_map_type handle_map_;

	enum Signals {
		SIGNAL_TX_DATA_READY = 1u << 0
	};

protected:
	size_t TransmitNoWait(const char* buffer, size_t nsize);
	size_t TransmitCanWait(const char* buffer, size_t nsize);
	size_t TransmitOnce(const char* buffer, size_t nsize);
	size_t ReceiveOnce(char* buffer, size_t nsize);
	bool WaitDataToTransmit();
	void SignalDataToTransmit();
	void StartTxThread();

private:
	const uint32_t tx_timeout_ = 50; // msec
	USBD_HandleTypeDef* usbd_ = nullptr;
	Ring<char, 2048> rx_ringbuf_;
	Ring<char, 2048> tx_ringbuf_;
	bool rx_initiated_ = true;
	osThreadId_t tx_thread_ = nullptr;
};

#endif /* _CDC_IFACE_H_ */
