#include "encoder_minas.h"

/*
 * panasonic_ma4_encoder.cpp
 *
 *  Created on: Oct 16, 2023
 *      Author: svetlio
 *      Author: Martin Stoilov
 */

#include "encoder_minas.h"
#include <cstring>
#include <assert.h>
#include <math.h>
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_usart.h"
#include "uartrpcserver.h"
#include "dcache.h"

extern UartRpcServer rpc_server;


EncoderMinas::EncoderMinas() :
		huart_(nullptr), crc_error_count_(0)
{
}

EncoderMinas::~EncoderMinas()
{
}

rexjson::property EncoderMinas::GetPropertyMap()
{
	rexjson::property props = rexjson::property_map({
		{"crc_errors", rexjson::property(&crc_error_count_, rexjson::property_get<decltype(crc_error_count_)>, rexjson::property_set<decltype(crc_error_count_)>)},
		{"status", rexjson::property(&status_, rexjson::property_get<decltype(status_)>, nullptr)},
		{"almc", rexjson::property(&almc_.as_byte_, rexjson::property_get<decltype(almc_.as_byte_)>, nullptr)},
		{"counter", rexjson::property(&counter_, rexjson::property_get<decltype(counter_)>, nullptr)},
		{"revolutions", rexjson::property(&revolutions_, rexjson::property_get<decltype(revolutions_)>, nullptr)},
		{"counter", rexjson::property(&counter_, rexjson::property_get<decltype(counter_)>, nullptr)},
		{"id", rexjson::property((void*)&encoder_id_, rexjson::property_get<decltype(encoder_id_)>, nullptr)},
		{"commandmode", rexjson::property((void*)&commandmode_, rexjson::property_get<decltype(commandmode_)>, rexjson::property_set<decltype(commandmode_)>)},
		{"position", rexjson::property(nullptr, 
			[&](void* ctx)->rexjson::value {
				return GetPosition();
			},
			nullptr)},
		{"time", rexjson::property(nullptr, 
			[&](void* ctx)->rexjson::value {
				return hrtimer.GetTimeElapsedMicroSec(t1_, t2_);
			}, 
			nullptr)},
	});
	return props;
}

void EncoderMinas::RegisterRpcMethods(const std::string& prefix)
{
	rpc_server.add(prefix, "resetF", rexjson::make_rpc_wrapper(this, &EncoderMinas::ResetErrorCodeF, "uint32_t EncoderMinas::ResetErrorCodeF()"));
	rpc_server.add(prefix, "resetB", rexjson::make_rpc_wrapper(this, &EncoderMinas::ResetErrorCodeB, "uint32_t EncoderMinas::ResetErrorCodeB()"));
	rpc_server.add(prefix, "resetE", rexjson::make_rpc_wrapper(this, &EncoderMinas::ResetErrorCodeE, "uint32_t EncoderMinas::ResetErrorCodeE()"));
	rpc_server.add(prefix, "reset9", rexjson::make_rpc_wrapper(this, &EncoderMinas::ResetErrorCode9, "uint32_t EncoderMinas::ResetErrorCode9()"));
	rpc_server.add(prefix, "reset_counter", rexjson::make_rpc_wrapper(this, &EncoderMinas::ResetPosition, "void EncoderMinas::ResetPosition()"));
	rpc_server.add(prefix, "readid", rexjson::make_rpc_wrapper(this, &EncoderMinas::ReadDeviceID, "void EncoderMinas::ReadDeviceId()"));
	rpc_server.add(prefix, "commandid4", rexjson::make_rpc_wrapper(this, &EncoderMinas::CommandId4, "void EncoderMinas::CommandId4()"));
	rpc_server.add(prefix, "commandid5", rexjson::make_rpc_wrapper(this, &EncoderMinas::CommandId5, "void EncoderMinas::CommandId5()"));
	rpc_server.add(prefix, "initialize", rexjson::make_rpc_wrapper(this, &EncoderMinas::Initialize, "void EncoderMinas::Initialize()"));
}

bool EncoderMinas::Attach(UART_HandleTypeDef* huart, DMA_TypeDef* dma, uint32_t rx_stream, uint32_t tx_stream)
{
	assert(huart);
	assert(huart_->hdmatx);
	assert(huart_->hdmarx);
	assert(dma);

	huart_ = huart;
	dma_ = dma;
	rx_stream_ = rx_stream;
	tx_stream_ = tx_stream;
	return Initialize();
}

void EncoderMinas::ReceiveCompleteCallback()
{
	t1_ = hrtimer.GetCounter();
	ParseResponse();
	t2_ = hrtimer.GetCounter();
}

void EncoderMinas::ParseResponse()
{
	if (update_.reply4_.ctrl_field_.as_byte == MA4_DATA_ID_4) {
		ParseReply4();
	} else if (update_.reply5_.ctrl_field_.as_byte == MA4_DATA_ID_5) {
		ParseReply5();
	} else if (update_.replyA_.ctrl_field_.as_byte == MA4_DATA_ID_A) {
		ParseReplyA();
	} else if (update_.reply9_.ctrl_field_.as_byte == MA4_DATA_ID_9) {
		ParseReply9BEF();
	} else if (update_.replyB_.ctrl_field_.as_byte == MA4_DATA_ID_B) {
		ParseReply9BEF();
	} else if (update_.replyE_.ctrl_field_.as_byte == MA4_DATA_ID_E) {
		ParseReply9BEF();
	} else if (update_.replyF_.ctrl_field_.as_byte == MA4_DATA_ID_F) {
		ParseReply9BEF();
	} else {
		crc_error_count_++;
	}
}

bool EncoderMinas::VerifyCrc(uint8_t* data, uint8_t size, uint8_t crc)
{
	uint8_t calc_crc = calc_crc_x8_1(data, size);
	if (crc != calc_crc) {
		++crc_error_count_;
		return false;
	}
	return true;
}

uint8_t EncoderMinas::calc_crc_x8_1(uint8_t* data, uint8_t size)
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

bool EncoderMinas::sendrecv_command_delay(uint8_t command, void* reply, size_t reply_size, size_t delay)
{
	bool ret = sendrecv_command(command, reply, reply_size);
	osDelay(delay);
	return ret;
}

bool EncoderMinas::sendrecv_command(uint8_t command, void* reply, size_t reply_size)
{
	if (!huart_)
		return false;
	if (!LL_USART_IsActiveFlag_TXE(huart_->Instance))
		return false;

	if (reply_size)
		*((uint8_t*)reply) = 0;
	LL_DMA_ConfigAddresses(dma_, rx_stream_, LL_USART_DMA_GetRegAddr(huart_->Instance, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t)reply, LL_DMA_GetDataTransferDirection(dma_, rx_stream_));
	LL_DMA_SetDataLength(dma_, rx_stream_, reply_size);
	LL_USART_EnableDMAReq_RX(huart_->Instance);
	LL_DMA_EnableIT_TC(dma_, rx_stream_);
	LL_DMA_EnableStream(dma_, rx_stream_);
	LL_USART_TransmitData8(huart_->Instance, command);
	if (crc_error_count_)
		return false;
	return true;
}

bool EncoderMinas::sendrecv_check_incommand(uint8_t command, void* reply, size_t reply_size)
{
	if (commandmode_)
		return false;
	return sendrecv_command(command, reply, reply_size);
}


bool EncoderMinas::CommandId9()
{
	return sendrecv_command_delay(MA4_DATA_ID_9, &update_.reply9_, sizeof(update_.reply9_));
}

bool EncoderMinas::CommandIdB()
{
	return sendrecv_command_delay(MA4_DATA_ID_B, &update_.replyB_, sizeof(update_.replyB_));
}

bool EncoderMinas::CommandIdE()
{
	return sendrecv_command_delay(MA4_DATA_ID_E, &update_.replyE_, sizeof(update_.replyE_));
}

bool EncoderMinas::CommandIdF()
{
	return sendrecv_command_delay(MA4_DATA_ID_F, &update_.replyF_, sizeof(update_.replyF_));
}

bool EncoderMinas::CommandId4()
{
	return sendrecv_command_delay(MA4_DATA_ID_4, &update_.reply4_, sizeof(update_.reply4_));
}

bool EncoderMinas::CommandId5()
{
	return sendrecv_command_delay(MA4_DATA_ID_5, &update_.reply5_, sizeof(update_.reply5_));
}

bool EncoderMinas::CommandIdA()
{
	return sendrecv_command_delay(MA4_DATA_ID_A, &update_.replyA_, sizeof(update_.replyA_));
}


bool EncoderMinas::CommandId4Ex()
{
	return sendrecv_check_incommand(MA4_DATA_ID_4, &update_.reply4_, sizeof(update_.reply4_));
}

bool EncoderMinas::CommandId5Ex()
{
	return sendrecv_check_incommand(MA4_DATA_ID_5, &update_.reply5_, sizeof(update_.reply5_));
}


/** Reset the encoder with the specified command id
 *
 * @param command Command code.
 */
void EncoderMinas::ResetWithCommand(uint8_t command)
{
	for (size_t i = 0; i < 10; i++) {
		osDelay(1);
		sendrecv_command(command, &update_.reply4_, sizeof(update_.reply4_));
	}
	osDelay(1);
}

void EncoderMinas::ResetErrorCode(uint8_t data_id)
{	
	ResetWithCommand(data_id);
}

void EncoderMinas::ResetErrorCode9()
{
	ResetErrorCode(MA4_DATA_ID_9);
}

void EncoderMinas::ResetErrorCodeF()
{
	ResetErrorCode(MA4_DATA_ID_F);
}

void EncoderMinas::ResetErrorCodeB()
{
	ResetErrorCode(MA4_DATA_ID_B);
}

void EncoderMinas::ResetErrorCodeE()
{
	ResetErrorCode(MA4_DATA_ID_E);
}

bool EncoderMinas::ParseReply4()
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

bool EncoderMinas::ParseReply5()
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

bool EncoderMinas::ParseReplyA()
{
	if (!VerifyCrc((uint8_t*) &update_.replyA_, sizeof(update_.replyA_) - 1, update_.replyA_.crc_))
		return false;
	encoder_id_ = update_.replyA_.encoder_id_;
	return true;
}

bool EncoderMinas::ParseReply9BEF()
{
	return ParseReply4();
}

uint32_t EncoderMinas::ReadDeviceID()
{
	encoder_id_ = 0;
	CommandIdA();
	osDelay(1);
	return encoder_id_;
}

uint32_t EncoderMinas::GetDeviceId()
{ 
	return encoder_id_; 
}

bool EncoderMinas::Initialize()
{
	commandmode_ = true;
	bool ret = true;
	osDelay(25);

	HAL_RS485Ex_Init(huart_, UART_DE_POLARITY_HIGH, 0, 0);

	status_ = 0;
	crc_error_count_ = 0;
	ResetErrorCodeE();
	uint32_t encoder_id = ReadDeviceID();
	if (!encoder_id)
		status_ |= status_e_not_detected;
	if (encoder_id == 0xa7) {
		cpr_bits_ = 23;
	} else if (encoder_id == 0x11) {
		cpr_bits_ = 17;
	} else {
		status_ |= status_e_not_initialized;
		ret = false;
	}
	commandmode_ = false;
	return ret;
}

void EncoderMinas::ResetPosition()
{
	ResetErrorCodeF();
}

uint32_t EncoderMinas::GetResolutionBits()
{
	return cpr_bits_;
}

uint32_t EncoderMinas::GetRevolutionBits()
{
	return 16UL;
}

uint64_t EncoderMinas::GetPosition()
{
	uint64_t ret = ((uint64_t) revolutions_ << cpr_bits_) | (uint64_t) counter_;
	return ret;
}

uint32_t EncoderMinas::GetIndexPosition()
{
	return 0;

}

uint32_t EncoderMinas::GetLastError()
{
	return 0;

}

uint32_t EncoderMinas::GetStatus()
{
	return status_ & ~status_mask;

}

bool EncoderMinas::Update()
{
	return CommandId5Ex();
}

void EncoderMinas::DisplayDebugInfo()
{
	static uint64_t new_position = 0;

	new_position = GetPosition();
		fprintf(stderr, "Minas(0x%x): %7.2f, Cnt: %10lu, Rev: %10lu, Pos: 0x%16llx, Status: %2lu (OS: %2u, FS: %2u, CE: %2u, OF: %2u, ME: %2u, SYD: %2u, BA: %2u ) (t1_to_t2: %5lu)\r\n",
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
				hrtimer.GetTimeElapsedMicroSec(t1_, t2_));
}

