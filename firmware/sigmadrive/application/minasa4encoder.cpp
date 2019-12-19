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
		thread_sendrecv_(0),
		mutex_sendrecv_(osMutexNew(NULL))
{
}

MinasA4Encoder::~MinasA4Encoder()
{
}

bool MinasA4Encoder::Detect()
{
	if (ResetAllErrors() == 0 && GetDeviceID() == 0xa7)
		resolution_ = (1 << 23);
	else if (ResetAllErrors() == 0 && GetDeviceID() == 0x11)
		resolution_ = (1 << 17);
	else
		return false;
	return true;
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
	return true;
}

void MinasA4Encoder::TransmitCompleteCallback()
{

}

void MinasA4Encoder::ReceiveCompleteCallback()
{
	EventThreadRxComplete();
}

bool MinasA4Encoder::ParseReply4(const MA4EncoderReply4& reply4, uint32_t& counter, MA4Almc& almc)
{
	uint8_t crc = calc_crc_x8_1((uint8_t*)&reply4, sizeof(reply4) - 1);
	if (crc != reply4.crc_) {
		fprintf(stderr, "WARNING: mismatched crc!\n");
		++error_count_;
		return false;
	}
	uint32_t abs_data =
			((uint32_t)reply4.absolute_data_[2] << 16) +
			((uint32_t)reply4.absolute_data_[1] << 8) +
			reply4.absolute_data_[0];
	counter = abs_data;
	almc = reply4.almc_;
	return true;
}

bool MinasA4Encoder::ParseReply9BEF(const MA4EncoderReply4& reply4, MA4Almc& almc)
{
	uint8_t crc = calc_crc_x8_1((uint8_t*)&reply4, sizeof(reply4) - 1);
	if (crc != reply4.crc_) {
		fprintf(stderr, "WARNING: mismatched crc!\n");
		++error_count_;
		return false;
	}
	almc = reply4.almc_;
	return true;
}

bool MinasA4Encoder::ParseReply5(const MA4EncoderReply5& reply5, uint32_t& status, uint32_t& counter, uint32_t& revolutions)
{
	uint8_t crc = calc_crc_x8_1((uint8_t*)&reply5, sizeof(reply5) - 1);
	if (crc != reply5.crc_) {
		fprintf(stderr, "WARNING: mismatched crc!\n");
		++error_count_;
		return false;
	}
	uint32_t abs_data =
			((uint32_t)reply5.absolute_data_[2] << 16) +
			((uint32_t)reply5.absolute_data_[1] << 8) +
			reply5.absolute_data_[0];
	status = (reply5.status_field_.ea1 << 1) | (reply5.status_field_.ea0);
	counter = abs_data;
	revolutions = *reinterpret_cast<const uint16_t*>(reply5.revolution_data_);
	return true;
}

bool MinasA4Encoder::ParseReplyA(const MA4EncoderReplyA& replyA, uint32_t& encoder_id)
{
	uint8_t crc = calc_crc_x8_1((uint8_t*)&replyA, sizeof(replyA) - 1);
	if (crc != replyA.crc_) {
		fprintf(stderr, "WARNING: mismatched crc!\n");
		++error_count_;
		return false;
	}
	encoder_id = replyA.encoder_id_;
	return true;
}

void MinasA4Encoder::EventThreadRxComplete()
{
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

uint32_t MinasA4Encoder::ResetAllErrors()
{
	uint8_t ret = 0;
	for (size_t i = 0; i < 10; i++) {
		if ((ret = ResetErrorCode(MA4_DATA_ID_E)) == 0)
			break;
		osDelay(500);
	}
	return ret;
}

uint32_t MinasA4Encoder::ResetErrorCode(uint8_t data_id)
{
	MA4Almc almc;
	uint32_t counter;
	MA4EncoderReply4 reply4;
	if (!ResetWithCommand(data_id, &reply4, sizeof(reply4))) {
		return -1;
	}
	if (!ParseReply4(reply4, counter, almc))
		return -1;
	return almc.as_byte_;
}

uint32_t MinasA4Encoder::ResetErrorCode9()
{
	return ResetErrorCode(MA4_DATA_ID_9);
}

uint32_t MinasA4Encoder::ResetErrorCodeF()
{
	return ResetErrorCode(MA4_DATA_ID_F);
}

uint32_t MinasA4Encoder::ResetErrorCodeB()
{
	return ResetErrorCode(MA4_DATA_ID_B);
}

uint32_t MinasA4Encoder::ResetErrorCodeE()
{
	return ResetErrorCode(MA4_DATA_ID_E);
}

bool MinasA4Encoder::UpdateWithCommand(uint8_t command, void* reply, size_t reply_size)
{
	bool ret = false;
	osMutexAcquire(mutex_sendrecv_, -1);
	thread_sendrecv_ = osThreadGetId();
	if (!sendrecv_command(command, reply, reply_size))
		goto error;
	if (!WaitEventRxComplete())
		goto error;
	ret = true;

error:
	osMutexRelease(mutex_sendrecv_);
	return ret;
}

bool MinasA4Encoder::ResetWithCommand(uint8_t command, void* reply, size_t reply_size)
{
	bool ret = false;
	osMutexAcquire(mutex_sendrecv_, -1);
	thread_sendrecv_ = osThreadGetId();
	for (size_t i = 0; i < 10; i++) {
		if (!sendrecv_command(command, reply, reply_size))
			goto error;
		if (!WaitEventRxComplete())
			goto error;
		osDelay(1);
	}
	ret = true;

error:
	osMutexRelease(mutex_sendrecv_);
	return ret;
}

uint32_t MinasA4Encoder::GetDeviceID()
{
	uint32_t encoder_id = -1;
	MA4EncoderReplyA replyA;
	if (!UpdateWithCommand(MA4_DATA_ID_A, &replyA, sizeof(replyA)))
		return -1;
	if (!ParseReplyA(replyA, encoder_id))
		return -1;
	return encoder_id;
}

uint32_t MinasA4Encoder::GetLastError()
{
	MA4Almc almc;
	uint32_t counter;
	MA4EncoderReply4 reply4;
	if (!UpdateWithCommand(MA4_DATA_ID_4, &reply4, sizeof(reply4))) {
		return -1;
	}
	if (!ParseReply4(reply4, counter, almc))
		return -1;
	return almc.as_byte_;
}

bool MinasA4Encoder::Update()
{
	MA4EncoderReply5 reply5;
	if (!UpdateWithCommand(MA4_DATA_ID_5, &reply5, sizeof(reply5)))
		return false;
	if (!ParseReply5(reply5, status_, counter_, revolutions_))
		return false;
	return true;
}

bool MinasA4Encoder::UpdateBegin()
{
	osMutexAcquire(mutex_sendrecv_, -1);
	thread_sendrecv_ = osThreadGetId();
	if (!sendrecv_command(MA4_DATA_ID_5, &reply5_, sizeof(reply5_))) {
		osMutexRelease(mutex_sendrecv_);
		return false;
	}
	return true;
}

bool MinasA4Encoder::UpdateEnd()
{
	bool ret = false;
	if (!WaitEventRxComplete())
		goto error;
	if (!ParseReply5(reply5_, status_, counter_, revolutions_))
		goto error;
	ret = true;

error:
	osMutexRelease(mutex_sendrecv_);
	return ret;
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
#define USE_LL_USART
#ifdef USE_LL_USART
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
