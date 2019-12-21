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
	StartTxThread();

}

size_t CdcIface::Transmit(const char* buffer, size_t nsize)
{
	size_t space_size = tx_ringbuf_.space_size();
	if ((space_size < tx_ringbuf_.capacity() / 2) || !nsize) {
		/* Nothing to send
		 *
		 * or
		 *
		 * If the queue is out of space
		 * just drop the write request.
		 */
		SignalDataToTransmit();
		return nsize;
	}

	nsize = std::min(tx_ringbuf_.write_size(), nsize);
	std::copy(buffer, buffer + nsize, tx_ringbuf_.get_write_ptr());
	tx_ringbuf_.write_update(nsize);
	SignalDataToTransmit();
	return nsize;
}

void CdcIface::RunTxLoop()
{
	for (;;) {
		if (WaitDataToTransmit()) {
			size_t nsize = tx_ringbuf_.read_size();
			while ((nsize = tx_ringbuf_.read_size()) != 0) {
				char* buffer = tx_ringbuf_.get_read_ptr();
				while (CDC_Transmit_FS((uint8_t*)buffer, nsize) == USBD_BUSY)
					osDelay(2);
				tx_ringbuf_.read_update(nsize);
			}
		}
	}

	tx_thread_ = 0;
}

static void RunTxLoopWrapper(void* ctx)
{
	reinterpret_cast<CdcIface*>(const_cast<void*>(ctx))->RunTxLoop();
}

bool CdcIface::WaitDataToTransmit()
{
	return (osThreadFlagsWait(SIGNAL_TX_DATA_READY, osFlagsWaitAll, tx_timeout_) == SIGNAL_TX_DATA_READY) ? true : false;
}

void CdcIface::SignalDataToTransmit()
{
	if (tx_thread_)
		osThreadFlagsSet(tx_thread_, SIGNAL_TX_DATA_READY);
}

void CdcIface::StartTxThread()
{
	osThreadAttr_t task_attributes;
	memset(&task_attributes, 0, sizeof(osThreadAttr_t));
	task_attributes.name = "CdcTxLoop";
	task_attributes.priority = (osPriority_t) osPriorityNormal;
	task_attributes.stack_size = 2048;
	tx_thread_ = osThreadNew(RunTxLoopWrapper, this, &task_attributes);


//	osThreadDef(RunTxLoopWrapper, osPriorityNormal, 0, 2048);
//	tx_thread_ = osThreadCreate(&os_thread_def_RunTxLoopWrapper, this);
}

size_t CdcIface::ReceiveOnce(char* buffer, size_t nsize)
{
	if (!nsize)
		return 0;
	size_t readsize = std::min(rx_ringbuf_.read_size(), nsize);
	std::copy(rx_ringbuf_.get_read_ptr(), rx_ringbuf_.get_read_ptr() + readsize, buffer);
	rx_ringbuf_.read_update(readsize);

	if (rx_initiated_ == false && rx_ringbuf_.space_size() > rx_ringbuf_.capacity()/2) {
		rx_initiated_ = true;
		CDC_Receive_Initiate();
	}
	return readsize;
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
