#ifndef _IENCODER_H_
#define _IENCODER_H_

#include <stdint.h>

class IEncoder {
public:
	virtual ~IEncoder() {};

	virtual void Start() = 0;
	virtual void Stop() = 0;
	virtual uint32_t GetCounter() = 0;
	virtual uint32_t GetMaxPosition() = 0;
	virtual uint32_t GetPosition() = 0;
	virtual uint32_t GetRevolutions() = 0;
	virtual uint32_t GetIndexPosition() = 0;
	virtual void ResetPosition() = 0;
	virtual float GetElectricPosition(uint32_t position, uint32_t motor_pole_pairs) = 0;
	virtual float GetMechanicalPosition(uint32_t position) = 0;
	virtual uint32_t GetLastError() = 0;
	virtual bool Update() = 0;
	virtual bool UpdateBegin() = 0;
	virtual bool UpdateEnd() = 0;
};

#endif /* _IENCODER_H_ */
