/*
 * MinasA4AbsEncoder.h
 *
 *  Created on: Mar 9, 2019
 *      Author: svetlio
 */

#ifndef MINAS_A4_ABS_ENCODER_H_
#define MINAS_A4_ABS_ENCODER_H_

#include <stdint.h>
#include <vector>
#include <string>
#include "units.h"
#include "usart.h"

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

static_assert(sizeof(MA4Almc) == 1, "ALMC must be 1 byte long");

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
	uint8_t manufacturer_reserved;
	MA4Almc almc_;
	uint8_t crc_;
};

static_assert(sizeof(MA4EncoderReplyA) == 9, "MA4EncoderReplyA must be 9 bytes long");

static const uint32_t MA4_ABS_ENCODER_RESOLUTION_BITS = 17;
static const uint32_t MA4_ABS_ENCODER_RESOLUTION = 1 << MA4_ABS_ENCODER_RESOLUTION_BITS;

class MinasA4AbsEncoder {
public:
	MinasA4AbsEncoder(USART& usart);
	~MinasA4AbsEncoder();
	bool init();
	uint16_t get_revolutions() const;
	float get_absolute_angle_deg() const;
	MA4Almc get_last_error() const;
	bool update();
	bool reset_all_errors();
	bool reset_single_revolution_data() { return reset_error_code(MA4_DATA_ID_F); }
	bool reset_multiple_revolution_data() { return reset_error_code(MA4_DATA_ID_B); }
	uint32_t get_error_count() const { return error_count_; }
	uint32_t get_counter() const { return (counter_ + MA4_ABS_ENCODER_RESOLUTION - MA4_ABS_ENCODER_RESOLUTION/20) % MA4_ABS_ENCODER_RESOLUTION; }

private:
	bool send_command(uint8_t command, void* reply, size_t reply_size);
	bool reset_error_code(uint8_t command);
	size_t read_usart_data(
				void* buf,
				size_t size,
				TimeSpan timeout);
	static uint8_t calc_crc_x8_1(uint8_t* data, uint8_t size);

private:
	USART& usart_;
	uint16_t revolutions_;
	float angle_deg_;
	uint32_t counter_;
	MA4Almc almc_;
	uint32_t error_count_;
};

#endif /* MINAS_A4_ABS_ENCODER_H_ */
