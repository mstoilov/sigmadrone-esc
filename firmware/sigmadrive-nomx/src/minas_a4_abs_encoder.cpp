/*
 * panasonic_ma4_encoder.cpp
 *
 *  Created on: Mar 9, 2019
 *      Author: svetlio
 */

#include "minas_a4_abs_encoder.h"

#include "sigmadrive.h"
#include "diag/Trace.h"
#include <cstring>

MinasA4AbsEncoder::MinasA4AbsEncoder(
		USART& usart) :
		usart_(usart),
		revolutions_(0),
		angle_deg_(0.0f),
		counter_(0),
		offset_(0),
		error_count_(0)
{
	almc_.as_byte_ = 0;
}

MinasA4AbsEncoder::~MinasA4AbsEncoder()
{
}

bool MinasA4AbsEncoder::init()
{
	MA4EncoderReplyA replyA;

	usart_.EnableDEMode();
	usart_.Enable();

	reset_all_errors();
	if (!send_command(MA4_DATA_ID_A, &replyA, sizeof(replyA))) {
		return false;
	}
	if (replyA.almc_.as_byte_ != 0) {
		return reset_all_errors();
	}
	return true;
}

uint16_t MinasA4AbsEncoder::get_revolutions() const
{
	return revolutions_;
}

float MinasA4AbsEncoder::get_absolute_angle_deg() const
{
	return angle_deg_;
}

MA4Almc MinasA4AbsEncoder::get_last_error() const
{
	return almc_;
}

bool MinasA4AbsEncoder::reset_all_errors()
{
	bool retval = reset_error_code(MA4_DATA_ID_B);
	if (!retval) {
		retval = reset_error_code(MA4_DATA_ID_E);
	}
	return retval;
}

bool MinasA4AbsEncoder::reset_error_code(uint8_t data_id)
{
#if 1
	MA4EncoderReplyB reply;
	almc_.as_byte_ = 0xff;
	if (send_command(data_id, &reply, sizeof(reply))) {
		almc_ = reply.almc_;
	}
	// Send the data_id 10 times, separated by at least 40 usecs
	for (uint32_t i = 0; i < 9; ++i) {
		HAL_Delay(1UL);
		if (send_command(data_id, &reply, sizeof(reply))) {
			almc_ = reply.almc_;
		}
	}
#endif
	return 0 == almc_.as_byte_;
}

bool MinasA4AbsEncoder::update()
{

	// first obtain angle with revolutions
	MA4EncoderReply5 reply5;
	almc_.as_byte_ = 0xff;
	if (!send_command(MA4_DATA_ID_5, &reply5, sizeof(reply5))) {
		++error_count_;
		return false;
	}

	if (reply5.ctrl_field_.as_byte != MA4_DATA_ID_5) {
		printf("PanasonicMA4Encoder received incorrect control field 0x%x, expected value was 0x%x\n",
				reply5.ctrl_field_.as_byte, MA4_DATA_ID_5);
		++error_count_;
		return false;
	}

	uint8_t crc = calc_crc_x8_1((uint8_t*)&reply5, sizeof(reply5)-1);
	if (crc != reply5.crc_) {
		printf("WARNING: mismatched crc!\n");
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
		if (send_command(MA4_DATA_ID_A, &replyA, sizeof(replyA))) {
			almc_ = replyA.almc_;
		}
	} else {
		almc_.as_byte_ = 0;
	}

	return true;
}

bool MinasA4AbsEncoder::send_command(uint8_t command, void* reply, size_t reply_size)
{
	size_t bytes_written = usart_.Write((char*)&command, sizeof(command));
	if (bytes_written != sizeof(command)) {
		printf("PanasonicMA4Encoder failed to send command 0x%x\n", command);
		return false;
	}
	memset(reply,0,reply_size);
	size_t bytes_read = read_usart_data(reply, reply_size, TimeSpan::from_milliseconds(1));
	if (bytes_read != reply_size) {
		printf("PanasonicMA4Encoder read only %u bytes, when sending command 0x%x\n", bytes_read, command);
		return false;
	}
	return true;
}

size_t MinasA4AbsEncoder::read_usart_data(
			void* buf,
			size_t size,
			TimeSpan timeout)
{
	uint8_t *bufptr = (uint8_t*)buf;
	int32_t cnt = 100000;
	while (size && --cnt >= 0) { // todo: use timeout
		size_t ret = usart_.ReadDMA((char*)bufptr, size);
		size -= ret;
		bufptr += ret;
	}
	return bufptr - (uint8_t*)buf;
}

uint8_t MinasA4AbsEncoder::calc_crc_x8_1(uint8_t* data, uint8_t size)
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
