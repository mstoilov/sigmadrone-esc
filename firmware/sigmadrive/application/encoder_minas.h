/*
 * encoder_minas.h
 *
 *  Created on: Oct 16, 2023
 *      Author: svetlio
 *      Author: Martin Stoilov
 *
 */

#ifndef ENCODER_MINAS_H_
#define ENCODER_MINAS_H_

#include "stm32f745xx.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_uart.h"

#include <stdint.h>
#include <vector>
#include <string>
#include <map>
#include "iencoder.h"
#include "hrtimer.h"
#include "rexjson/rexjsonproperty.h"

#include "cmsis_os2.h"

extern HRTimer hrtimer;

//
// Definitions for Panasonic Minas A4 17 bit absolute encoder
//

// ==============================================================================
//
//  Structure of Control Field
//  ----0-----0-----1-----0-----cc0-----cc1-----cc2-----cc3-----cc4-----1
//  StartBit|   Sync Code    |          Data ID Code        | Parity | StopBit
//
//   	  		  Parity     Data ID Code        Sync Code
// 			  	 ---cc4---cc3---cc2---cc1---cc0---0---1---0
// Data ID 4         1     0     1     0     0    0   1   0
// Data ID 5         0     0     1     0     1    0   1   0
// Data ID A         0     1     0     1     0    0   1   0
// -------------------------------------------------------
// Data ID 9         0     1     0     0     1    0   1   0
// Data ID B         1     1     0     1     1    0   1   0
// Data ID E         1     1     1     1     0    0   1   0
// Data ID F         0     1     1     1     1    0   1   0
//
// ===============================================================================


class EncoderMinas : public IEncoder {
public:

	static const uint8_t MA4_DATA_ID_4 = 0xa2; //10100010
	static const uint8_t MA4_DATA_ID_5 = 0x2a; //00101010
	static const uint8_t MA4_DATA_ID_A = 0x52; //01010010
	static const uint8_t MA4_DATA_ID_9 = 0x4a; //01001010
	static const uint8_t MA4_DATA_ID_B = 0xda; //11011010
	static const uint8_t MA4_DATA_ID_E = 0xf2; //11110010
	static const uint8_t MA4_DATA_ID_F = 0x7a; //01111010

	static const uint8_t MA4_CF_SYNC_CODE 		= 0x2;
	static const uint8_t MA4_CF_SYNC_CODE_MASK 	= 0x7;

	struct MA4ControlField {
		union {
			struct {
				uint8_t sync_code: 3; // always 010b
				uint8_t cc0: 1;
				uint8_t cc1: 1;
				uint8_t cc2: 1;
				uint8_t cc3: 1;
				uint8_t cc4: 1;
			};
			uint8_t as_byte;
		};
	};


	struct MA4EncoderRequest {
		MA4ControlField ctrl_field_;
	};

	// ===================================================
	// Reply from encoder
	// | CF | SF | DF0 | DF1 | ... | DF5 | CRC
	//    |    |    |                       |
	//    |    |    |                        -- 1 byte
	//    |    |    |
	//    |    |     -- Data Fields - up to 6 bytes
	//    |    |
	//    |     -- Status Field - 1 byte
	//    |
	//     -- Control Field - 1 byte
	// ===================================================

	struct MA4StatusField {
		union {
			struct {
				uint8_t dd: 4;       // always 0
				uint8_t ea0: 1;      // System Down
				uint8_t ea1: 1;      // Multiple rev error, batt. alarm, full absolute status,
									// counter overflow
				uint8_t reserved: 2; // always 0
			};
			uint8_t as_byte;
		};
	};

	struct MA4Almc {
		union {
			struct {
				uint8_t overspeed_ 					: 1; // OS
				uint8_t full_abs_status_ 			: 1; // FS
				uint8_t count_error_ 				: 1; // CE
				uint8_t counter_overflow_ 			: 1; // OF
				uint8_t reserved_ 					: 1; // always 0
				uint8_t multiple_revolution_error_ 	: 1; // ME
				uint8_t system_down_ 				: 1; // SYD
				uint8_t battery_alarm_ 				: 1; // BA
			};
			uint8_t as_byte_;
		};
	};

	static_assert(sizeof(uint8_t) == 1, "ALMC must be 1 byte long");

	struct MA4EncoderReply4 {
		MA4ControlField ctrl_field_;
		MA4StatusField status_field_;
		uint8_t absolute_data_[3];
		MA4Almc almc_; // encoder error
		uint8_t crc_;
	};

	typedef MA4EncoderReply4 MA4EncoderReply9;
	typedef MA4EncoderReply4 MA4EncoderReplyB;
	typedef MA4EncoderReply4 MA4EncoderReplyE;
	typedef MA4EncoderReply4 MA4EncoderReplyF;

	static_assert(sizeof(MA4EncoderReply4) == 7, "MA4EncoderReply4 must be 7 bytes long");

	struct MA4EncoderReply5 {
		MA4ControlField ctrl_field_;
		MA4StatusField status_field_;
		uint8_t absolute_data_[3];
		uint8_t revolution_data_[2];
		uint8_t not_used_;
		uint8_t crc_;
	};

	static_assert(sizeof(MA4EncoderReply5) == 9, "MA4EncoderReply5 must be 9 bytes long");

	struct MA4EncoderReplyA {
		MA4ControlField ctrl_field_;
		MA4StatusField status_field_;
		uint8_t absolute_data_[3];
		uint8_t encoder_id_; // fixed to 0x11
		uint8_t maker_id_;
		MA4Almc almc_;
		uint8_t crc_;
	};

	static_assert(sizeof(MA4EncoderReplyA) == 9, "MA4EncoderReplyA must be 9 bytes long");

	struct MA4Update {
		union {
			MA4EncoderReplyA replyA_;
			MA4EncoderReply4 reply4_;
			MA4EncoderReply5 reply5_;
			MA4EncoderReply9 reply9_;
			MA4EncoderReplyB replyB_;
			MA4EncoderReplyE replyE_;
			MA4EncoderReplyF replyF_;
		};
	};

	static const uint32_t status_e_not_initialized = (1 << 16);
	static const uint32_t status_e_not_detected = (1 << 17);
	static const uint32_t status_mask = (1 << 0);

	EncoderMinas();
	~EncoderMinas();
	bool Attach(UART_HandleTypeDef* usart, DMA_TypeDef* dma, uint32_t rx_stream, uint32_t tx_stream);
	uint32_t GetDeviceId();
	void ReceiveCompleteCallback();

	rexjson::property GetPropertyMap();
	void RegisterRpcMethods(const std::string& prefix);


protected:
	void ParseResponse();
	bool VerifyCrc(uint8_t* data, uint8_t size, uint8_t crc);
	bool sendrecv_command(uint8_t command, void* reply, size_t reply_size);
	bool sendrecv_set_incommand(uint8_t command, void* reply, size_t reply_size);
	bool sendrecv_check_incommand(uint8_t command, void* reply, size_t reply_size);	
	static uint8_t calc_crc_x8_1(uint8_t* data, uint8_t size);

	uint32_t ReadDeviceID();
	bool ParseReply4();
	bool ParseReply5();
	bool ParseReplyA();
	bool ParseReply9BEF();
	void ResetWithCommand(uint8_t command);

	bool CommandId9();
	bool CommandIdB();
	bool CommandIdE();
	bool CommandIdF();
	bool CommandId4();
	bool CommandId5();
	bool CommandIdA();
	bool CommandId4Ex();
	bool CommandId5Ex();
	void ResetErrorCode(uint8_t data_id);
	void ResetErrorCodeF();
	void ResetErrorCodeB();
	void ResetErrorCodeE();
	void ResetErrorCode9();

public:
	/**
	 * Initialize the hardware
	 * @return Returning true indicates the encoder is successfully initialized
	 * and ready to be used. Return false if there is an error.
	 */
	virtual bool Initialize();

	/** Set the current position of the
	 * encoder as 0 reference.
	 */
	virtual void ResetPosition();

	/** Return the number of bits used by the hardware
	 * to store the counts of one full rotation (cpr_bits).
	 */
	virtual uint32_t GetResolutionBits();

	/** Return the number of bits used by the hardware
	 * to store the maximum supported revolution counts.
	 */
	virtual uint32_t GetRevolutionBits();

	/** Combined revolutions and position counts
	 * Position = (revolutions << resolution_bits) | position
	 */
	virtual uint64_t GetPosition();

	/** Return the index position
	 */
	virtual uint32_t GetIndexPosition();

	/** Return the last error.
	 */
	virtual uint32_t GetLastError();

	/** Return encoder status. If this value is
	 * different than 0, the encoder is not working
	 * correctly.
	 */
	virtual uint32_t GetStatus();

	/** Intended to begin the hardware communication
	 * to retrieve the current encoder values.
	 */
	virtual bool Update();

	/** Dump debug internal information
	 *
	 */
	virtual void DisplayDebugInfo();


	UART_HandleTypeDef* huart_;
	DMA_TypeDef* dma_;
	uint32_t rx_stream_;
	uint32_t tx_stream_;
	uint32_t encoder_id_ = 0;           /**< Cached encoder id, set by the last call to GetDeviceID() */
	uint32_t cpr_bits_ = 17;            /**< Counts per rotation (encoder resolution) bits */
	uint32_t status_ = 0;               /**< Encoder status bits, received from the encoder */
	uint32_t counter_ = 0;              /**< Encoder counter, defines the encoder position within one revolution  */
	uint32_t revolutions_ = 16;         /**< Number of revolutions */
	uint32_t crc_error_count_ = 0;      /**< The count of the communication errors, seen so far */
	MA4Almc almc_;                      /**< Holds the alarm bits received from the encoder */
	MA4Update update_;                  /**< Holds the encoder response */
	bool incommand_ = false;            /**< Skip updates while this flag is true. sendrecv_command_ex will return false immediately */

	volatile uint32_t t1_ = 0;
	volatile uint32_t t2_ = 0;

};

#endif // ENCODER_MINAS_H_
