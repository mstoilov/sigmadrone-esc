
#ifndef _ISERVODRIVE_H_
#define _ISERVODRIVE_H_

#include "iencoder.h"
#include "ipwmgenerator.h"

class IServoDrive {
public:
	virtual ~IServoDrive() {}
	IServoDrive() {}

	virtual IEncoder* GetEncoder() const = 0;
	virtual IPwmGenerator* GetPwmGenerator() const = 0;
	virtual void Start() = 0;
	virtual void Stop() = 0;
	virtual bool IsStarted() = 0;
};

#endif /* _ISERVODRIVE_H_ */
