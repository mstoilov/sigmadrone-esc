/*
 * usb_cdc.cpp
 *
 *  Created on: Sep 16, 2019
 *      Author: mstoilov
 */

#include <assert.h>
#include "cdc_iface.h"

CdcIface::handle_map_type CdcIface::handle_map_;


CdcIface::CdcIface()
	: rx_initiated_(true)
{
	// TODO Auto-generated constructor stub

}

CdcIface::~CdcIface()
{
	// TODO Auto-generated destructor stub
}

void CdcIface::Attach(USBD_HandleTypeDef* usbd)
{
	usbd_ = usbd;
	assert(handle_map_.find(usbd) == handle_map_.end());
	handle_map_[usbd_] = this;

}

size_t CdcIface::Transmit(const char* buffer, size_t nsize)
{
	if (nsize > 0xFFFF)
		nsize = 0xFFFF;
	if (CDC_Transmit_FS((uint8_t*)buffer, nsize) == USBD_BUSY)
		return 0;
	return nsize;
}

size_t CdcIface::Receive(char* buffer, size_t nsize)
{
	size_t readsize = std::min(rx_ringbuf_.read_size(), nsize);
	std::copy(rx_ringbuf_.get_read_ptr(), rx_ringbuf_.get_read_ptr() + readsize, buffer);
	rx_ringbuf_.read_update(readsize);

	if (rx_initiated_ == false && rx_ringbuf_.space_size() > rx_ringbuf_.capacity()/2) {
		rx_initiated_ = true;
		CDC_Receive_Initiate();
	}
	return readsize;
}

int8_t CdcIface::ReceiveComplete(uint8_t* buf, uint32_t len)
{
	assert(len < rx_ringbuf_.space_size());

	size_t offset = 0;
	while (len) {
		size_t copy_size = std::min((size_t)len, rx_ringbuf_.write_size());
		std::copy(buf + offset, buf + offset + copy_size, rx_ringbuf_.get_write_ptr());
		rx_ringbuf_.write_update(copy_size);
		len -= copy_size;
		offset += copy_size;
	}

	if (rx_ringbuf_.space_size() > rx_ringbuf_.capacity()/2) {
		rx_initiated_ = true;
		CDC_Receive_Initiate();
	} else {
		rx_initiated_ = false;
	}

	return USBD_OK;
}

extern "C"
int8_t cdc_iface_rx_complete(USBD_HandleTypeDef* usbd, uint8_t* buf, uint32_t len)
{
	CdcIface* ptr = CdcIface::handle_map_[usbd];
	if (ptr) {
		return ptr->ReceiveComplete(buf, len);
	}

	return USBD_FAIL;
}
