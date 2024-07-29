#ifndef _IENCODER_H_
#define _IENCODER_H_

#include <stdint.h>

class IEncoder {
public:
	IEncoder() = default;
	virtual ~IEncoder() { }

	/**
	 * Initialize the hardware
	 * @return Returning true indicates the encoder is successfully initialized
	 * and ready to be used. Return false if there is an error.
	 */
	virtual bool Initialize() = 0;

	/** Set the current position of the
	 * encoder as 0 reference.
	 */
	virtual void ResetPosition() = 0;

	/** Return the number of bits used by the hardware
	 * to store the counts of one full rotation (cpr_bits).
	 */
	virtual uint32_t GetResolutionBits() = 0;

	/** Return the number of bits used by the hardware
	 * to store the maximum supported revolution counts.
	 */
	virtual uint32_t GetRevolutionBits() = 0;

	/** Combined revolutions and position counts
	 * Position = (revolutions << resolution_bits) | position
	 */
	virtual uint64_t GetPosition() = 0;

	/** Return the index position
	 */
	virtual uint32_t GetIndexPosition() = 0;

	/** Return the last error.
	 */
	virtual uint32_t GetLastError() = 0;

	/** Return encoder status. If this value is
	 * different than 0, the encoder is not working
	 * correctly.
	 */
	virtual uint32_t GetStatus() = 0;

	/** Intended to begin the hardware communication
	 * to retrieve the current encoder values.
	 */
	virtual bool Update() = 0;

	/** Dump debug internal information
	 *
	 */
	virtual void DisplayDebugInfo() = 0;
};

#endif /* _IENCODER_H_ */
