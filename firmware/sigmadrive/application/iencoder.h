#ifndef _IENCODER_H_
#define _IENCODER_H_

#include <stdint.h>
#include "property.h"

class IEncoder {
public:
	virtual ~IEncoder() {};

	virtual void Start() = 0;
	virtual void Stop() = 0;
	virtual uint32_t GetMaxPosition() = 0;
	virtual uint32_t GetPosition() = 0;
	virtual int32_t GetIndexPosition() = 0;
	virtual void ResetPosition(uint32_t) = 0;
	virtual float GetElectricPosition() = 0;
	virtual float GetMechanicalPosition() = 0;
	virtual uint32_t GetMotorPolePairs() = 0;
	virtual void SetMotorPolePairs(uint32_t motor_pole_pairs) = 0;
	virtual rexjson::property GetProperties() = 0;
public:


};

#endif /* _IENCODER_H_ */
