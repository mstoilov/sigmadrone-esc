#ifndef _IENCODER_H_
#define _IENCODER_H_

#include <stdint.h>

class IEncoder {
public:
	IEncoder() = default;
	virtual ~IEncoder() { }

	/*
	 * Set the current position of the
	 * encoder as 0 reference.
	 */
	virtual void ResetPosition() { }

	/*
	 * Return the number of bits used by the hardware.
	 * The maximum position the encoder can return is:
	 * (1 << resolution_bits) - 1
	 */
	virtual uint32_t GetResolutionBits() { return 16; }

	/*
	 * Get Encoder (rotational) position from 0 to (1 << encoder_bits)
	 *
	 */
	virtual uint32_t GetPosition() { return 0; }

	/*
	 * Combined revolutios and position
	 * AbsPos = (revolutions << encoder_bits) | position
	 */
	virtual uint64_t GetAbsolutePosition() { return 0; }

	/*
	 * Combined revolutios and position max value
	 * AbsPos = (1 << (revolution_bits + position_bits))
	 */
	virtual uint64_t GetAbsolutePositionMax() { return (1ULL << 32); }


	/*
	 * Return the number of completed revolutions.
	 */
	virtual uint32_t GetRevolutions() { return 0; }

	/*
	 * Return the index position
	 */
	virtual uint32_t GetIndexPosition() { return 0; }


	/*
	 * Calculate the electric angle for the specified position
	 * in Rad
	 */
	virtual float GetElectricPosition(uint64_t position, uint32_t motor_pole_pairs) { return 0; }

	/*
	 * Calculate the mechanical angle for the specified position
	 * in Rad
	 */
	virtual float GetMechanicalPosition(uint64_t position) { return 0; }

	/*
	 * Return the last error.
	 */
	virtual uint32_t GetLastError() { return 0; }

	/*
	 * Intended to begin the hardware communication
	 * to retrieve the current encoder values.
	 */
	virtual bool Update() { return true; }
};

#endif /* _IENCODER_H_ */
