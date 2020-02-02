#ifndef _IENCODER_H_
#define _IENCODER_H_

#include <stdint.h>

class IEncoder {
public:
	virtual ~IEncoder() {};

	/*
	 * Set the current position of the
	 * encoder as 0 reference.
	 */
	virtual void ResetPosition() = 0;

	/*
	 * Return the number of bits used by the hardware.
	 * The maximum position the encoder can return is:
	 * (1 << resolution_bits) - 1
	 */
	virtual uint32_t GetResolutionBits() = 0;

	/*
	 * Get Encoder (rotational) position from 0 to (1 << encoder_bits)
	 *
	 */
	virtual uint32_t GetPosition() = 0;

	/*
	 * Combined revolutios and position
	 * AbsPos = (revolutions << encoder_bits) | position
	 */
	virtual uint64_t GetAbsolutePosition() = 0;

	/*
	 * Combined revolutios and position max value
	 * AbsPos = (1 << (revolution_bits + position_bits))
	 */
	virtual uint64_t GetAbsolutePositionMax() = 0;


	/*
	 * Return the number of completed revolutions.
	 */
	virtual uint32_t GetRevolutions() = 0;

	/*
	 * Return the index position
	 */
	virtual uint32_t GetIndexPosition() = 0;


	/*
	 * Calculate the electric angle for the specified position
	 * in Rad
	 */
	virtual float GetElectricPosition(uint64_t position, uint32_t motor_pole_pairs) = 0;

	/*
	 * Calculate the mechanical angle for the specified position
	 * in Rad
	 */
	virtual float GetMechanicalPosition(uint64_t position) = 0;

	/*
	 * Return the last error.
	 */
	virtual uint32_t GetLastError() = 0;

	/*
	 * Intended to begin the hardware communication
	 * to retrieve the current encoder values.
	 */
	virtual bool Update() = 0;
};

#endif /* _IENCODER_H_ */
