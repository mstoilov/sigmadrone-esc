/*
 * usb_cdc.h
 *
 *  Created on: Sep 16, 2019
 *      Author: mstoilov
 */

#ifndef _CDC_IFACE_H_
#define _CDC_IFACE_H_

#include <map>

#include "ring.h"
#include "usbd_cdc_if.h"


class CdcIface {
public:
	using handle_map_type = std::map<USBD_HandleTypeDef*, CdcIface*>;

	CdcIface();
	virtual ~CdcIface();
	void Attach(USBD_HandleTypeDef* usbd);
	int8_t ReceiveComplete(uint8_t* buf, uint32_t len);
	size_t Transmit(const char* buffer, size_t nsize);
	size_t Receive(char* buffer, size_t nsize);

	static handle_map_type handle_map_;

private:
	USBD_HandleTypeDef* usbd_;
	Ring<char, 4096> rx_ringbuf_;
	bool rx_initiated_;
};

#endif /* _CDC_IFACE_H_ */
