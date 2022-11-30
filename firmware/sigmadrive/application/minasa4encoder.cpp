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
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_usart.h"
#include "dcache.h"


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
		huart_(nullptr), crc_error_count_(0)
{
}

MinasA4Encoder::~MinasA4Encoder()
{
}

#if 0
rexjson::property MinasA4Encoder::GetPropertyMap()
{
	rexjson::property props = rexjson::property_map({
		{"crc_errors", rexjson::property(&crc_error_count_, rexjson::property_access::readonly)},
	});
	return props;
}

void MinasA4Encoder::RegisterRpcMethods(const std::string& prefix)
{
	rpc_server.add(prefix, "resetF", rexjson::make_rpc_wrapper(this, &MinasA4Encoder::ResetErrorCodeF, "uint32_t MinasA4Encoder::ResetErrorCodeF()"));
	rpc_server.add(prefix, "resetB", rexjson::make_rpc_wrapper(this, &MinasA4Encoder::ResetErrorCodeB, "uint32_t MinasA4Encoder::ResetErrorCodeB()"));
	rpc_server.add(prefix, "resetE", rexjson::make_rpc_wrapper(this, &MinasA4Encoder::ResetErrorCodeE, "uint32_t MinasA4Encoder::ResetErrorCodeE()"));
	rpc_server.add(prefix, "reset9", rexjson::make_rpc_wrapper(this, &MinasA4Encoder::ResetErrorCode9, "uint32_t MinasA4Encoder::ResetErrorCode9()"));
	rpc_server.add(prefix, "reset_counter", rexjson::make_rpc_wrapper(this, &MinasA4Encoder::ResetPosition, "void MinasA4Encoder::ResetPosition()"));
}
#endif

bool MinasA4Encoder::Attach(UART_HandleTypeDef* huart, DMA_TypeDef* dma, uint32_t rx_stream, uint32_t tx_stream)
{
	assert(huart);
	assert(huart_->hdmatx);
	assert(huart_->hdmarx);
	assert(dma);

	huart_ = huart;
	dma_ = dma;
	rx_stream_ = rx_stream;
	tx_stream_ = tx_stream;
	assert(handle_map_.find(huart_) == handle_map_.end());
	handle_map_[huart_] = this;
	huart_->RxCpltCallback = ::minasa4_rx_complete;
	return Initialize();
}

void MinasA4Encoder::TransmitCompleteCallback()
{

}

void MinasA4Encoder::ReceiveCompleteCallback()
{
	--in_command_;
	UpdateEnd();
	t2_ = hrtimer.GetCounter();
	update_time_ms_ = hrtimer.GetTimeElapsedMicroSec(t1_, t2_);
}

bool MinasA4Encoder::CommandId9()
{
	return sendrecv_command_ex(MA4_DATA_ID_9, &update_.reply9_, sizeof(update_.reply9_));
}

bool MinasA4Encoder::CommandIdB()
{
	return sendrecv_command_ex(MA4_DATA_ID_B, &update_.replyB_, sizeof(update_.replyB_));
}

bool MinasA4Encoder::CommandIdE()
{
	return sendrecv_command_ex(MA4_DATA_ID_E, &update_.replyE_, sizeof(update_.replyE_));
}

bool MinasA4Encoder::CommandIdF()
{
	return sendrecv_command_ex(MA4_DATA_ID_F, &update_.replyF_, sizeof(update_.replyF_));
}

bool MinasA4Encoder::CommandId4()
{
	return sendrecv_command_ex(MA4_DATA_ID_4, &update_.reply4_, sizeof(update_.reply4_));
}

bool MinasA4Encoder::CommandId5()
{
	return sendrecv_command_ex(MA4_DATA_ID_5, &update_.reply5_, sizeof(update_.reply5_));
}

bool MinasA4Encoder::CommandIdA()
{
	return sendrecv_command_ex(MA4_DATA_ID_A, &update_.replyA_, sizeof(update_.replyA_));
}


bool MinasA4Encoder::ParseReply4()
{
	if (!VerifyCrc((uint8_t*) &update_.reply4_, sizeof(update_.reply4_) - 1, update_.reply4_.crc_))
		return false;
	uint32_t abs_data = ((uint32_t) update_.reply4_.absolute_data_[2] << 16)
			+ ((uint32_t) update_.reply4_.absolute_data_[1] << 8)
			+ update_.reply4_.absolute_data_[0];
	status_ = (update_.reply4_.status_field_.ea1 << 1) | (update_.reply4_.status_field_.ea0);
	counter_ = abs_data;
	almc_ = update_.reply4_.almc_;
	return true;
}

bool MinasA4Encoder::ParseReply5()
{
	if (!VerifyCrc((uint8_t*) &update_.reply5_, sizeof(update_.reply5_) - 1, update_.reply5_.crc_))
		return false;
	uint32_t abs_data = ((uint32_t) update_.reply5_.absolute_data_[2] << 16)
			+ ((uint32_t) update_.reply5_.absolute_data_[1] << 8)
			+ update_.reply5_.absolute_data_[0];
	status_ = (update_.reply5_.status_field_.ea1 << 1) | (update_.reply5_.status_field_.ea0);
	counter_ = abs_data;
	revolutions_ = *reinterpret_cast<const uint16_t*>(update_.reply5_.revolution_data_);
	return true;
}

bool MinasA4Encoder::ParseReplyA()
{
	if (!VerifyCrc((uint8_t*) &update_.replyA_, sizeof(update_.replyA_) - 1, update_.replyA_.crc_))
		return false;
	encoder_id_ = update_.replyA_.encoder_id_;
	return true;
}

bool MinasA4Encoder::ParseReply9BEF()
{
	return ParseReply4();
}

/** Reset the encoder with the specified command id
 *
 * @param command Command code.
 * @return true if successful, otherwise false
 */
bool MinasA4Encoder::ResetWithCommand(uint8_t command)
{
	__disable_irq();
	if (in_command_) {
		__enable_irq();
		return false;
	}
	/*
	 * Get one reference, that will be released
	 * at the end.
	 */
	++in_command_;
	__enable_irq();

	for (size_t i = 0; i < 10; i++) {
		/*
		 * Get reference, it will be released by the
		 * completion handler. But we still hold one
		 * reference.
		 */
		++in_command_;
		if (!sendrecv_command(command, &update_.reply4_, sizeof(update_.reply4_))) {
			in_command_ = 0;
			return false;
		}
		osDelay(1);
	}
	/*
	 * Release the only reference. At this point we
	 * must hold 0 references.
	 */
	--in_command_;
	if (in_command_) {
		in_command_ = 0;
		return false;
	}
	return true;
}

uint32_t MinasA4Encoder::ResetErrorCode(uint8_t data_id)
{
	if (!ResetWithCommand(data_id)) {
		return -1;
	}
	if (!ParseReply4())
		return -1;
	return almc_.as_byte_;
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

uint32_t MinasA4Encoder::ReadDeviceID()
{
	encoder_id_ = 0;
	if (!CommandIdA())
		goto error;
	osDelay(1);
	if (!ParseReplyA())
		goto error;

error:
	return encoder_id_;
}

uint32_t MinasA4Encoder::GetLastError()
{
	if (!CommandId4())
		goto error;
	osDelay(1);
	if (!ParseReply4())
		goto error;

error:
	return almc_.as_byte_;
}

uint32_t MinasA4Encoder::GetStatus()
{
	return status_ & ~status_mask;
}

bool MinasA4Encoder::Update()
{
	uint32_t prev_t1 = t1_;
	t1_ = hrtimer.GetCounter();
	t1_to_t1_ = hrtimer.GetTimeElapsedMicroSec(prev_t1, t1_);
	return CommandId5();
}

bool MinasA4Encoder::UpdateEnd()
{
	if (update_.reply4_.ctrl_field_.as_byte == MA4_DATA_ID_4) {
		return ParseReply4();
	} else if (update_.reply5_.ctrl_field_.as_byte == MA4_DATA_ID_5) {
		return ParseReply5();
	}
	return false;
}

bool MinasA4Encoder::sendrecv_command(uint8_t command, void* reply, size_t reply_size)
{
	if (!huart_)
		return false;
	if (!LL_USART_IsActiveFlag_TXE(huart_->Instance))
		return false;

	LL_DMA_ConfigAddresses(dma_, rx_stream_, LL_USART_DMA_GetRegAddr(huart_->Instance, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t)reply, LL_DMA_GetDataTransferDirection(dma_, rx_stream_));
	LL_DMA_SetDataLength(dma_, rx_stream_, reply_size);
	LL_USART_EnableDMAReq_RX(huart_->Instance);
	LL_DMA_EnableIT_TC(dma_, rx_stream_);
	LL_DMA_EnableStream(dma_, rx_stream_);
	LL_USART_TransmitData8(huart_->Instance, command);
	return true;
}

bool MinasA4Encoder::sendrecv_command_ex(uint8_t command, void* reply, size_t reply_size)
{
	if (!huart_)
		return false;
	if (!LL_USART_IsActiveFlag_TXE(huart_->Instance))
		return false;

	__disable_irq();
	if (in_command_) {
		__enable_irq();
		return false;
	}
	++in_command_;
	__enable_irq();
	LL_DMA_ConfigAddresses(dma_, rx_stream_, LL_USART_DMA_GetRegAddr(huart_->Instance, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t)reply, LL_DMA_GetDataTransferDirection(dma_, rx_stream_));
	LL_DMA_SetDataLength(dma_, rx_stream_, reply_size);
	LL_USART_EnableDMAReq_RX(huart_->Instance);
	LL_DMA_EnableIT_TC(dma_, rx_stream_);
	LL_DMA_EnableStream(dma_, rx_stream_);
	LL_USART_TransmitData8(huart_->Instance, command);
	return true;
}


bool MinasA4Encoder::VerifyCrc(uint8_t* data, uint8_t size, uint8_t crc)
{
	return true;
	uint8_t calc_crc = calc_crc_x8_1(data, size);
	if (crc != calc_crc) {
		++crc_error_count_;
		return false;
	}
	return true;
}

uint8_t MinasA4Encoder::calc_crc_x8_1(uint8_t* data, uint8_t size)
{
	uint8_t j;
	uint8_t carry;
	uint8_t crc;

	crc = 0;

	while (size-- > 0) {
		crc ^= *data++;
		for (j = 8; j != 0; j--) {
			carry = crc & 0x80;
			crc <<= 1;
			if (carry != 0) {
				crc ^= 0x01; /* Polynomial X^8 + 1  */
			}
		}
	}
	return (crc & 0x00FF);
}

uint32_t MinasA4Encoder::GetResolutionBits()
{
	return cpr_bits_;
}

uint32_t MinasA4Encoder::GetRevolutionBits()
{
	return 16UL;
}

bool MinasA4Encoder::Initialize()
{
	status_ = 0;
	in_command_ = 0;
	crc_error_count_ = 0;
	ResetErrorCodeE();
	uint32_t encoder_id = ReadDeviceID();
	CommandId5();
	if (!encoder_id)
		status_ |= status_e_not_detected;
	if (encoder_id == 0xa7) {
		cpr_bits_ = 23;
	} else if (encoder_id == 0x11) {
		cpr_bits_ = 17;
	} else {
		status_ |= status_e_not_initialized;
		return false;
	}
	return true;
}

void MinasA4Encoder::ResetPosition()
{
}

uint64_t MinasA4Encoder::GetPosition()
{
	uint64_t ret = ((uint64_t) revolutions_ << cpr_bits_) | (uint64_t) counter_;
	return ret;
}

uint32_t MinasA4Encoder::GetIndexPosition()
{
	return 0;
}

void MinasA4Encoder::DisplayDebugInfo()
{
	static uint64_t old_counter = 0, new_position = 0;

	new_position = GetPosition();
	if (new_position != old_counter || status_) {
		fprintf(stderr, "Minas(0x%x): %7.2f, Cnt: %10lu, Rev: %10lu, Pos: 0x%16llx, Status: %2lu (OS: %2u, FS: %2u, CE: %2u, OF: %2u, ME: %2u, SYD: %2u, BA: %2u ) (UpdT: %5lu, t1_to_t1: %5lu)\r\n",
				(int)encoder_id_,
				counter_ * 360.0f / (1 << cpr_bits_),
				counter_,
				revolutions_,
				new_position,
				status_,
				almc_.overspeed_,
				almc_.full_abs_status_,
				almc_.count_error_,
				almc_.counter_overflow_,
				almc_.multiple_revolution_error_,
				almc_.system_down_,
				almc_.battery_alarm_,
				update_time_ms_,
				t1_to_t1_);
		old_counter = new_position;
	}
}

