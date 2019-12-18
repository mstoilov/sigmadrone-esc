/*
 * panasonic_ma4_encoder.cpp
 *
 *  Created on: Mar 9, 2019
 *      Author: svetlio
 */

#include "minasa4encoder.h"
#include <cstring>
#include <assert.h>
#include <math.h>
#include "stm32f7xx_ll_usart.h"

MinasA4Encoder::handle_map_type MinasA4Encoder::handle_map_;

extern "C" void minasa4_tx_complete(UART_HandleTypeDef* huart)
{
	MinasA4Encoder* ptr = MinasA4Encoder::handle_map_[huart];
	if (ptr) {
		ptr->TransmitCompleteCallback();
	}
}

extern "C" void minasa4_rx_complete(UART_HandleTypeDef* huart)
{
	MinasA4Encoder* ptr = MinasA4Encoder::handle_map_[huart];
	if (ptr) {
		ptr->ReceiveCompleteCallback();
	}
}

MinasA4Encoder::MinasA4Encoder() :
		huart_(nullptr),
		offset_(0),
		error_count_(0),
		thread_sendrecv_(0)
{
	almc_.as_byte_ = 0;
}

MinasA4Encoder::~MinasA4Encoder()
{
}

bool MinasA4Encoder::Attach(UART_HandleTypeDef* huart)
{
	assert(huart);
	assert(huart_->hdmatx);
	assert(huart_->hdmarx);

	huart_ = huart;
	assert(handle_map_.find(huart_) == handle_map_.end());
	handle_map_[huart_] = this;
	huart_->RxCpltCallback = ::minasa4_rx_complete;
	if (ResetAllErrors() == 0 && GetDeviceID() == 0xa7)
		resolution_ = (1 << 23);
	return true;
}

void MinasA4Encoder::TransmitCompleteCallback()
{

}

void MinasA4Encoder::ReceiveCompleteCallback()
{
	EventThreadRxComplete();
}

void MinasA4Encoder::ParseReply4()
{
	uint8_t crc = calc_crc_x8_1((uint8_t*)&reply_.reply4_, sizeof(reply_.reply4_) - 1);
	if (crc != reply_.reply4_.crc_) {
		fprintf(stderr, "WARNING: mismatched crc!\n");
		++error_count_;
	}
	almc_ = reply_.reply4_.almc_;
	uint32_t abs_data =
			((uint32_t)reply_.reply4_.absolute_data_[2] << 16) +
			((uint32_t)reply_.reply4_.absolute_data_[1] << 8) +
			reply_.reply4_.absolute_data_[0];
	status_ = (reply_.reply4_.status_field_.ea1 << 1) | (reply_.reply4_.status_field_.ea0);
	counter_ = abs_data;
}

void MinasA4Encoder::ParseReply9BEF()
{
	uint8_t crc = calc_crc_x8_1((uint8_t*)&reply_.reply4_, sizeof(reply_.reply4_) - 1);
	if (crc != reply_.reply4_.crc_) {
		fprintf(stderr, "WARNING: mismatched crc!\n");
		++error_count_;
	}
	almc_ = reply_.reply4_.almc_;
}

void MinasA4Encoder::ParseReply5()
{
	uint8_t crc = calc_crc_x8_1((uint8_t*)&reply_.reply5_, sizeof(reply_.reply5_) - 1);
	if (crc != reply_.reply5_.crc_) {
		fprintf(stderr, "WARNING: mismatched crc!\n");
		++error_count_;
	}
	uint32_t abs_data =
			((uint32_t)reply_.reply5_.absolute_data_[2] << 16) +
			((uint32_t)reply_.reply5_.absolute_data_[1] << 8) +
			reply_.reply5_.absolute_data_[0];
	status_ = (reply_.reply5_.status_field_.ea1 << 1) | (reply_.reply5_.status_field_.ea0);
	counter_ = abs_data;
	revolutions_ = *reinterpret_cast<uint16_t*>(reply_.reply5_.revolution_data_);
}

void MinasA4Encoder::ParseReplyA()
{
	uint8_t crc = calc_crc_x8_1((uint8_t*)&reply_.replyA_, sizeof(reply_.replyA_) - 1);
	if (crc != reply_.replyA_.crc_) {
		fprintf(stderr, "WARNING: mismatched crc!\n");
		++error_count_;
	}
	uint32_t abs_data =
			((uint32_t)reply_.replyA_.absolute_data_[2] << 16) +
			((uint32_t)reply_.replyA_.absolute_data_[1] << 8) +
			reply_.replyA_.absolute_data_[0];
	status_ = (reply_.replyA_.status_field_.ea1 << 1) | (reply_.replyA_.status_field_.ea0);
	almc_ = reply_.replyA_.almc_;
	counter_ = abs_data;
	encoder_id_ = reply_.replyA_.encoder_id_;
}


void MinasA4Encoder::EventThreadRxComplete()
{
	switch (reply_.reply4_.ctrl_field_.as_byte)
	{
	case MA4_DATA_ID_4:
		ParseReply4();
		break;
	case MA4_DATA_ID_5:
		ParseReply5();
		break;
	case MA4_DATA_ID_A:
		ParseReplyA();
		break;
	case MA4_DATA_ID_9:
	case MA4_DATA_ID_B:
	case MA4_DATA_ID_E:
	case MA4_DATA_ID_F:
		ParseReply9BEF();
		break;
	default:

		break;
	}

	rx_complete_ = true;
	if (thread_sendrecv_)
		osThreadFlagsSet(thread_sendrecv_, EVENT_FLAG_RX_COMPLETE);
}

bool MinasA4Encoder::WaitEventRxComplete(uint32_t timeout)
{
	uint32_t status = osThreadFlagsWait(EVENT_FLAG_RX_COMPLETE, osFlagsWaitAll, timeout);
	signal_time_ms_ = hrtimer.GetTimeElapsedMicroSec(t1_, hrtimer.GetCounter());
	bool ret = ((status & EVENT_FLAG_RX_COMPLETE) == EVENT_FLAG_RX_COMPLETE) ? true : false;
	if (!ret)
		fprintf(stderr, "WaitEventRxComplete timeout\n");
	return ret;
}

uint32_t MinasA4Encoder::GetLastError()
{
	return almc_.as_byte_;
}

uint8_t MinasA4Encoder::ResetAllErrors()
{
	uint8_t ret = 0;
	for (size_t i = 0; i < 10; i++) {
		if ((ret = ResetErrorCode(MA4_DATA_ID_E)) == 0)
			break;
		osDelay(500);
	}
	return ret;
}

uint8_t MinasA4Encoder::ResetErrorCode(uint8_t data_id)
{
	if (!ResetWithCommand(data_id, &reply_.reply4_, sizeof(reply_.reply4_))) {
		return -1;
	}
	return almc_.as_byte_;
}

uint8_t MinasA4Encoder::ResetErrorCode9()
{
	return ResetErrorCode(MA4_DATA_ID_9);
}

uint8_t MinasA4Encoder::ResetErrorCodeF()
{
	return ResetErrorCode(MA4_DATA_ID_F);
}

uint8_t MinasA4Encoder::ResetErrorCodeB()
{
	return ResetErrorCode(MA4_DATA_ID_B);
}

uint8_t MinasA4Encoder::ResetErrorCodeE()
{
	return ResetErrorCode(MA4_DATA_ID_E);
}

bool MinasA4Encoder::WaitForUpdate()
{
	return WaitEventRxComplete(2);
}

bool MinasA4Encoder::UpdateWithCommand(osThreadId_t wakeupThreadId, uint8_t command, void* reply, size_t reply_size)
{
	__disable_irq();
	if (!rx_complete_ || !reset_complete_) {
		__enable_irq();
		return false;
	}
	rx_complete_ = false;
	__enable_irq();
	thread_sendrecv_ = wakeupThreadId;
	if (!sendrecv_command(command, reply, reply_size)) {
		rx_complete_ = true;
		return false;
	}
	return true;
}

/*
 * All this reset_complete_ bull***t is required to make sure
 * no other update or reset will interfere while the current reset command
 * is being transmitted 10 times (as prescribed in the specification).
 */
bool MinasA4Encoder::ResetWithCommand(uint8_t command, void* reply, size_t reply_size)
{
	__disable_irq();
	if (!rx_complete_ || !reset_complete_) {
		__enable_irq();
		return false;
	}
	reset_complete_ = false;
	__enable_irq();
	thread_sendrecv_ = osThreadGetId();
	for (size_t i = 0; i < 10; i++) {
		if (!sendrecv_command(command, reply, reply_size))
			goto error;
		if (!WaitForUpdate())
			goto error;
		osDelay(1);
	}
	reset_complete_ = true;
	return true;

error:
	reset_complete_ = true;
	return false;
}

uint32_t MinasA4Encoder::GetDeviceID()
{
	if (!UpdateWithCommand(osThreadGetId(), MA4_DATA_ID_A, &reply_.replyA_, sizeof(reply_.replyA_)))
		return -1;
	if (!WaitForUpdate())
		return -1;
	return encoder_id_;
}

bool MinasA4Encoder::Update(void* wakeupThreadId)
{
	return UpdateWithCommand((osThreadId_t)wakeupThreadId, MA4_DATA_ID_5, &reply_.reply5_, sizeof(reply_.reply5_));
}

bool MinasA4Encoder::sendrecv_command(uint8_t command, void* reply, size_t reply_size)
{
	if (!huart_)
		return false;
	t1_ = hrtimer.GetCounter();
	if (HAL_UART_Receive_DMA(huart_, (uint8_t*)reply, reply_size) != HAL_OK) {
		fprintf(stderr, "%s: PanasonicMA4Encoder failed to receive reply for command 0x%x\n", __FUNCTION__, command);
		HAL_UART_Abort(huart_);
		return false;
	}
#if USE_LL_USART
	if (!LL_USART_IsActiveFlag_TXE(huart_->Instance)) {
		fprintf(stderr, "%s: PanasonicMA4Encoder failed to send command 0x%x\n", __FUNCTION__, command);
		HAL_UART_Abort(huart_);
		return false;
	}
	LL_USART_TransmitData8(huart_->Instance, command);
#else
	if (HAL_UART_Transmit_DMA(huart_, (uint8_t*)&command, sizeof(command)) != HAL_OK) {
		fprintf(stderr, "%s: PanasonicMA4Encoder failed to send command 0x%x\n", __FUNCTION__, command);
		return false;
	}
#endif
	return true;
}

uint8_t MinasA4Encoder::calc_crc_x8_1(uint8_t* data, uint8_t size)
{
  uint8_t j;
  uint8_t carry;
  uint8_t crc;

  crc = 0;

  while (size-- > 0)
  {
    crc ^= *data++;
    for (j = 8; j != 0; j--)
    {
      carry = crc & 0x80;
      crc <<= 1;
      if (carry != 0)
      {
        crc ^= 0x01;  /* Polynomial X^8 + 1  */
      }
    }
  }
  return (crc & 0x00FF);
}

void MinasA4Encoder::ResetPosition()
{
	uint32_t counter = GetCounter();
	if (counter == (uint32_t)-1)
		return;
	offset_ = counter;
}

uint32_t MinasA4Encoder::GetCounter()
{
	return counter_;
}


uint32_t MinasA4Encoder::GetPosition()
{
	uint32_t counter = GetCounter();
	if (counter == (uint32_t)-1)
		return -1;
	return (counter + resolution_ - offset_) % resolution_;
}

uint32_t MinasA4Encoder::GetRevolutions()
{
	return revolutions_;
}


uint32_t MinasA4Encoder::GetIndexPosition()
{
	return 0;
}

float MinasA4Encoder::GetElectricPosition(uint32_t position, uint32_t motor_pole_pairs)
{
	return 2.0f * M_PI * (position % (resolution_ / motor_pole_pairs)) / (resolution_ / motor_pole_pairs);
}

float MinasA4Encoder::GetMechanicalPosition(uint32_t position)
{
	return 2.0f * M_PI * (position % (resolution_)) / (resolution_);
}
