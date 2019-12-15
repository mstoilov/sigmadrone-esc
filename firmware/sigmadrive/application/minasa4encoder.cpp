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
		revolutions_(0),
		angle_deg_(0.0f),
		counter_(0),
		offset_(0),
		error_count_(0),
		update_thread_(0),
		event_dma_(osEventFlagsNew(NULL))
{
	almc_.as_byte_ = 0;
}

MinasA4Encoder::~MinasA4Encoder()
{
}

bool MinasA4Encoder::Attach(UART_HandleTypeDef* huart)
{
	MA4EncoderReplyA replyA;

	assert(huart);
	assert(huart_->hdmatx);
	assert(huart_->hdmarx);

	huart_ = huart;
	assert(handle_map_.find(huart_) == handle_map_.end());
	handle_map_[huart_] = this;
	huart_->TxCpltCallback = ::minasa4_tx_complete;
	huart_->RxCpltCallback = ::minasa4_rx_complete;


	reset_all_errors();
	update();
	if (!sendrecv_command(MA4_DATA_ID_A, &replyA, sizeof(replyA))) {
		return false;
	}
//	if (replyA.almc_.as_byte_ != 0) {
//		return reset_all_errors();
//	}
	return true;
}

void MinasA4Encoder::TransmitCompleteCallback()
{

}

void MinasA4Encoder::ReceiveCompleteCallback()
{
	rx_completed_ = true;
	EventThreadRxComplete();
}

bool MinasA4Encoder::WaitEventRxComplete(uint32_t timeout)
{
	uint32_t status = osEventFlagsWait(event_dma_, EVENT_FLAG_RX_COMPLETE, osFlagsWaitAny, timeout);
	return ((status & EVENT_FLAG_RX_COMPLETE) == EVENT_FLAG_RX_COMPLETE) ? true : false;
}

uint16_t MinasA4Encoder::get_revolutions() const
{
	return revolutions_;
}

float MinasA4Encoder::get_absolute_angle_deg() const
{
	return angle_deg_;
}

MA4Almc MinasA4Encoder::get_last_error() const
{
	return almc_;
}

bool MinasA4Encoder::reset_all_errors()
{
	bool retval = reset_error_code(MA4_DATA_ID_B);
	if (!retval) {
		retval = reset_error_code(MA4_DATA_ID_E);
	}
	return retval;
}

bool MinasA4Encoder::reset_error_code(uint8_t data_id)
{
#if 1
	MA4EncoderReplyB reply;
	almc_.as_byte_ = 0xff;
	if (sendrecv_command(data_id, &reply, sizeof(reply))) {
		almc_ = reply.almc_;
	}
	// Send the data_id 10 times, separated by at least 40 usecs
	for (uint32_t i = 0; i < 9; ++i) {
		HAL_Delay(1UL);
		if (sendrecv_command(data_id, &reply, sizeof(reply))) {
			almc_ = reply.almc_;
		}
	}
#endif
	return 0 == almc_.as_byte_;
}

bool MinasA4Encoder::update()
{

	// first obtain angle with revolutions
	MA4EncoderReply5 reply5;
	almc_.as_byte_ = 0xff;
	if (!sendrecv_command(MA4_DATA_ID_5, &reply5, sizeof(reply5))) {
		++error_count_;
		return false;
	}

	if (reply5.ctrl_field_.as_byte != MA4_DATA_ID_5) {
		fprintf(stderr, "PanasonicMA4Encoder received incorrect control field 0x%x, expected value was 0x%x\n",
				reply5.ctrl_field_.as_byte, MA4_DATA_ID_5);
		++error_count_;
		return false;
	}

	uint8_t crc = calc_crc_x8_1((uint8_t*)&reply5, sizeof(reply5)-1);
	if (crc != reply5.crc_) {
		fprintf(stderr, "WARNING: mismatched crc!\n");
		++error_count_;
		return false;
	}

	revolutions_ = *reinterpret_cast<uint16_t*>(reply5.revolution_data_);
	uint32_t abs_data = ((uint32_t)reply5.absolute_data_[2] << 16) +
			((uint32_t)reply5.absolute_data_[1] << 8) +
			reply5.absolute_data_[0];
	counter_ = abs_data & 0x1ffff;
	angle_deg_ = (float)(counter_) * 360.0f / (float)MA4_ABS_ENCODER_RESOLUTION;

	if (reply5.status_field_.ea0 || reply5.status_field_.ea1) {
		MA4EncoderReplyA replyA;
		if (sendrecv_command(MA4_DATA_ID_A, &replyA, sizeof(replyA))) {
			almc_ = replyA.almc_;
		}
	} else {
		almc_.as_byte_ = 0;
	}

	return true;
}

bool MinasA4Encoder::sendrecv_command(uint8_t command, void* reply, size_t reply_size)
{
	memset(reply, 0, reply_size);
	rx_completed_ = false;
	osEventFlagsClear(event_dma_, EVENT_FLAG_RX_COMPLETE);
	if (HAL_UART_Receive_DMA(huart_, (uint8_t*)reply, reply_size) != HAL_OK) {
		fprintf(stderr, "PanasonicMA4Encoder failed to receive reply for command 0x%x\n", command);
		return false;
	}
	if (HAL_UART_Transmit_DMA(huart_, (uint8_t*)&command, sizeof(command)) != HAL_OK) {
		fprintf(stderr, "PanasonicMA4Encoder failed to send command 0x%x\n", command);
		return false;
	}
	return WaitEventRxComplete(2);
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

void MinasA4Encoder::RunUpdateLoop()
{
	for (;;) {
		update();
		osDelay(1);
	}
}

static void RunUpdateLoopWrapper(void* ctx)
{
	reinterpret_cast<MinasA4Encoder*>(const_cast<void*>(ctx))->RunUpdateLoop();
}


void MinasA4Encoder::StartUpdateThread()
{
	osThreadAttr_t task_attributes;
	memset(&task_attributes, 0, sizeof(osThreadAttr_t));
	task_attributes.name = " MinasA4EncoderThread";
	task_attributes.priority = (osPriority_t) osPriorityNormal;
	task_attributes.stack_size = 2048;
	update_thread_ = osThreadNew(RunUpdateLoopWrapper, this, &task_attributes);
}

void MinasA4Encoder::ResetPosition(uint32_t position)
{
	reset_initial_counter_offset();
	offset_ += position;
}

uint32_t MinasA4Encoder::GetPosition()
{
	uint32_t max_position = GetMaxPosition();
	return (get_counter() + max_position) % max_position;
}

int32_t MinasA4Encoder::GetIndexPosition()
{
	return 0;
}

float MinasA4Encoder::GetElectricPosition(uint32_t motor_pole_pairs)
{
	uint32_t max_position = GetMaxPosition();
	return 2.0f * M_PI * (GetPosition() % (max_position / motor_pole_pairs)) / (max_position / motor_pole_pairs);
}

float MinasA4Encoder::GetMechanicalPosition()
{
	uint32_t max_postion = GetMaxPosition();
	return 2.0f * M_PI * (GetPosition() % (max_postion)) / (max_postion);
}
