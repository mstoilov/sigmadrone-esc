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
        huart_(nullptr), error_count_(0)
{
}

MinasA4Encoder::~MinasA4Encoder()
{
}

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
    return true;
}

void MinasA4Encoder::TransmitCompleteCallback()
{

}

void MinasA4Encoder::ReceiveCompleteCallback()
{
    UpdateEnd();
    t2_ = hrtimer.GetCounter();
    update_time_ms_ = hrtimer.GetTimeElapsedMicroSec(t1_, t2_);
}

bool MinasA4Encoder::ParseReply4(const MA4EncoderReply4& reply4,
        uint32_t& status, uint32_t& counter, MA4Almc& almc)
{
    if (!VerifyCrc((uint8_t*) &reply4, sizeof(reply4) - 1, reply4.crc_))
        return false;
    uint32_t abs_data = ((uint32_t) reply4.absolute_data_[2] << 16)
            + ((uint32_t) reply4.absolute_data_[1] << 8)
            + reply4.absolute_data_[0];
    status = (reply4.status_field_.ea1 << 1) | (reply4.status_field_.ea0);
    counter = abs_data;
    almc = reply4.almc_;
    return true;
}

bool MinasA4Encoder::ParseReply9BEF(const MA4EncoderReply4& reply4, MA4Almc& almc)
{
    if (!VerifyCrc((uint8_t*) &reply4, sizeof(reply4) - 1, reply4.crc_))
        return false;
    almc = reply4.almc_;
    return true;
}

bool MinasA4Encoder::ParseReply5(const MA4EncoderReply5& reply5,
        uint32_t& status, uint32_t& counter, uint32_t& revolutions)
{
    if (!VerifyCrc((uint8_t*) &reply5, sizeof(reply5) - 1, reply5.crc_))
        return false;
    uint32_t abs_data = ((uint32_t) reply5.absolute_data_[2] << 16)
            + ((uint32_t) reply5.absolute_data_[1] << 8)
            + reply5.absolute_data_[0];
    status = (reply5.status_field_.ea1 << 1) | (reply5.status_field_.ea0);
    counter = abs_data;
    revolutions = *reinterpret_cast<const uint16_t*>(reply5.revolution_data_);
    return true;
}

bool MinasA4Encoder::ParseReplyA(const MA4EncoderReplyA& replyA, uint32_t& encoder_id)
{
    if (!VerifyCrc((uint8_t*) &replyA, sizeof(replyA) - 1, replyA.crc_))
        return false;
    encoder_id = replyA.encoder_id_;
    return true;
}

bool MinasA4Encoder::ResetWithCommand(uint8_t command, void* reply,
        size_t reply_size)
{
    maintenance_ = 1;
    osDelay(2);
    bool ret = false;
    for (size_t i = 0; i < 10; i++) {
        if (!sendrecv_command(command, reply, reply_size))
            goto error;
        osDelay(1);
    }
    ret = true;

    error: maintenance_ = 0;
    return ret;
}

uint32_t MinasA4Encoder::ResetErrorCode(uint8_t data_id)
{
    MA4Almc almc;
    uint32_t counter;
    uint32_t status;
    MA4EncoderReply4 reply4;
    if (!ResetWithCommand(data_id, &reply4, sizeof(reply4))) {
        return -1;
    }
    if (!ParseReply4(reply4, status, counter, almc))
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

uint32_t MinasA4Encoder::GetDeviceID()
{
    uint32_t encoder_id = -1;
    MA4EncoderReplyA replyA;
    maintenance_ = 1;
    osDelay(2);
    replyA.crc_ = 0;
    if (!sendrecv_command(MA4_DATA_ID_A, &replyA, sizeof(replyA)))
        goto error;
    osDelay(2);
    if (!ParseReplyA(replyA, encoder_id))
        goto error;

    error: maintenance_ = 0;
    encoder_id_  = encoder_id;
    return encoder_id;
}

uint32_t MinasA4Encoder::GetLastError()
{
    uint32_t error = 1;
    MA4Almc almc;
    uint32_t counter;
    uint32_t status;
    MA4EncoderReply4 reply4;
    maintenance_ = 1;
    osDelay(2);
    reply4.crc_ = 0;
    if (!sendrecv_command(MA4_DATA_ID_4, &reply4, sizeof(reply4)))
        goto error;
    osDelay(2);
    if (!ParseReply4(reply4, status, counter, almc))
        goto error;
    error = almc.as_byte_;

    error: maintenance_ = 0;
    return error;
}

bool MinasA4Encoder::UpdateId4()
{
    if (maintenance_)
        return false;
    update_.reply4_.crc_ = 0;
    if (!sendrecv_command(MA4_DATA_ID_4, &update_.reply4_, sizeof(update_.reply4_)))
        return false;
    return true;
}

bool MinasA4Encoder::UpdateId5()
{
    if (maintenance_)
        return false;
    update_.reply5_.crc_ = 0;
    if (!sendrecv_command(MA4_DATA_ID_5, &update_.reply5_,sizeof(update_.reply5_)))
        return false;
    return true;
}

bool MinasA4Encoder::Update()
{
    uint32_t prev_t1 = t1_;
    t1_ = hrtimer.GetCounter();
    t1_to_t1_ = hrtimer.GetTimeElapsedMicroSec(prev_t1, t1_);
    return UpdateId5();
}

bool MinasA4Encoder::UpdateEnd()
{
    if (update_.reply4_.ctrl_field_.as_byte == MA4_DATA_ID_4) {
        return ParseReply4(update_.reply4_, status_, counter_, almc_);
    } else if (update_.reply5_.ctrl_field_.as_byte == MA4_DATA_ID_5) {
        return ParseReply5(update_.reply5_, status_, counter_, revolutions_);
    }
    return false;
}

bool MinasA4Encoder::sendrecv_command(uint8_t command, void* reply, size_t reply_size)
{
    if (!huart_)
        return false;

#define USE_LL_USART
#ifdef USE_LL_USART
    LL_DMA_ConfigAddresses(dma_, rx_stream_, LL_USART_DMA_GetRegAddr(huart_->Instance, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t)reply, LL_DMA_GetDataTransferDirection(dma_, rx_stream_));
    LL_DMA_SetDataLength(dma_, rx_stream_, reply_size);
    LL_USART_EnableDMAReq_RX(huart_->Instance);
    LL_DMA_EnableIT_TC(dma_, rx_stream_);
    LL_DMA_EnableStream(dma_, rx_stream_);

#else
    HAL_StatusTypeDef status;
    if ((status = HAL_UART_Receive_DMA(huart_, (uint8_t*) reply, reply_size)) != HAL_OK) {
        fprintf(stderr, "%s: PanasonicMA4Encoder failed to receive reply for command 0x%x, with status %u\n", __FUNCTION__, command,
                status);
        HAL_UART_Abort(huart_);
        return false;
    }
#endif
    if (!LL_USART_IsActiveFlag_TXE(huart_->Instance)) {
        fprintf(stderr, "%s: PanasonicMA4Encoder failed to send command 0x%x\n", __FUNCTION__, command);
        HAL_UART_Abort(huart_);
        return false;
    }
    LL_USART_TransmitData8(huart_->Instance, command);
    return true;
}

bool MinasA4Encoder::VerifyCrc(uint8_t* data, uint8_t size, uint8_t crc)
{
    uint8_t calc_crc = calc_crc_x8_1(data, size);
    if (crc != calc_crc) {
        if (error_count_ % 100 == 0)
            fprintf(stderr, "WARNING: mismatched crc!\n");
        ++error_count_;
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
    ResetErrorCodeE();
    if (GetDeviceID() == 0xa7) {
        cpr_bits_ = 23;
    } else if (GetDeviceID() == 0x11) {
        cpr_bits_ = 17;
    } else {
        return false;
    }
    return true;
}

void MinasA4Encoder::ResetPosition()
{
//    ResetErrorCodeF();
//    ResetErrorCodeB();
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
        fprintf(stderr, "Minas(0x%x): %7.2f, Cnt: %10lu, Rev: %10lu, Pos: 0x%16llx, Status: %2lu (OS: %2u, FS: %2u, CE: %2u, OF: %2u, ME: %2u, SYD: %2u, BA: %2u ) (UpdT: %5lu, t1_to_t1: %5lu)\n",
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

