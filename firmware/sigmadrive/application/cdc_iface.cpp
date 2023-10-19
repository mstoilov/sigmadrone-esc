/*
 * usb_cdc.cpp
 *
 *  Created on: Sep 16, 2019
 *      Author: mstoilov
 */

#include <assert.h>
#include "string.h"
#include "cdc_iface.h"
#include "dcache.h"

CdcIface::handle_map_type CdcIface::handle_map_;


CdcIface::CdcIface()
{
	// TODO Auto-generated constructor stub
	cdcevt_id_ = osEventFlagsNew(nullptr);

}

CdcIface::~CdcIface()
{
	// TODO Auto-generated destructor stub
	osEventFlagsDelete(cdcevt_id_);
}

void CdcIface::Attach(USBD_HandleTypeDef* usbd, bool start_tx_thread)
{
	usbd_ = usbd;
	assert(handle_map_.find(usbd) == handle_map_.end());
	handle_map_[usbd_] = this;
}

size_t CdcIface::TransmitOnce(const char* buffer, size_t nsize)
{
	size_t ret = std::min(nsize, (size_t)64);
	while (CDC_Transmit_FS((uint8_t*)buffer, ret) == USBD_BUSY)
		osDelay(1);
	return ret;
}

size_t CdcIface::Transmit(const char* buffer, size_t nsize)
{
	const char *p = buffer;
	size_t size = nsize;
	size_t offset = 0;
	size_t ret = 0;
	while (size) {
		ret = TransmitOnce(p + offset, size);
		offset += ret;
		size -= ret;
	}
	return nsize;
}

size_t CdcIface::Transmit(const std::string& str)
{
	return Transmit(str.c_str(), str.size());
}

bool CdcIface::WaitDataToReceive()
{
	return (osEventFlagsWait(cdcevt_id_, SIGNAL_RX_DATA_READY, osFlagsWaitAny, osWaitForever) == SIGNAL_RX_DATA_READY) ? true : false;
}

void CdcIface::SignalDataToReceive()
{
	osEventFlagsSet(cdcevt_id_, SIGNAL_RX_DATA_READY);
}

size_t CdcIface::ReceiveOnce(char* buffer, size_t nsize)
{
	if (!nsize)
		return 0;
	size_t recvsize = 0;
again:
	recvsize = std::min(rx_ringbuf_.read_size(), nsize);
	if (!recvsize) {
		if (rx_initiated_ == false && rx_ringbuf_.space_size() > rx_ringbuf_.capacity()/2) {
			rx_initiated_ = true;
			CDC_Receive_Initiate();
		}
		WaitDataToReceive();
		goto again;
	}
	const char *src = rx_ringbuf_.get_read_ptr();
	for (size_t i = 0; i < recvsize; i++) {
		buffer[i] = src[i];
		if (buffer[i] == '\n')
			recvsize = i + 1;
	}
	rx_ringbuf_.read_update(recvsize);
	return recvsize;
}

size_t CdcIface::Receive(char* buffer, size_t nsize)
{
	ssize_t recvsiz = 0;
	size_t ret = 0;
	size_t offset = 0;
	while (nsize) {
		recvsiz = ReceiveOnce(buffer + offset, nsize);
		if (recvsiz <= 0)
			break;
		ret += recvsiz;
		offset += recvsiz;
		nsize -= recvsiz;
	}
	return ret;
}

size_t CdcIface::ReceiveLine(char* buffer, size_t nsize)
{
	ssize_t recvsiz = 0;
	size_t ret = 0;
	size_t offset = 0;
	while (nsize) {
		recvsiz = ReceiveOnce(buffer + offset, nsize);
		if (recvsiz <= 0)
			break;
		if (buffer[offset + recvsiz - 1] == '\n')
			return ret + recvsiz;
		ret += recvsiz;
		offset += recvsiz;
		nsize -= recvsiz;
	}
	return ret;
}

std::string CdcIface::GetLine()
{
	char rxbuffer[64];
	size_t ret = 0;
	std::string recv;

again:
	ret = ReceiveOnce(rxbuffer, sizeof(rxbuffer) - 1);
	if (ret < 0)
		return recv;
	recv += std::string(rxbuffer, ret);
	if (rxbuffer[ret - 1] != '\n')
		goto again;
	return recv;
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
		SignalDataToReceive();
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
