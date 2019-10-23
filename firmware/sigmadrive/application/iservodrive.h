
#ifndef _ISERVODRIVE_H_
#define _ISERVODRIVE_H_

#include "iencoder.h"
#include "ipwmgenerator.h"

class IServoDrive {
public:
	virtual ~IServoDrive();
	IServoDrive();

/* Set Functions */
	virtual void SetEncoder(IEncoder *encoder);
	virtual void SetPwmGenerator(IPwmGenerator *encoder);


	virtual IEncoder* GetEncoder();
	virtual IPwmGenerator* GetPwmGenerator();


	virtual void Start() = 0;
	virtual void Stop() = 0;
	virtual bool IsStarted() = 0;

public:
	IEncoder *encoder_ = nullptr;
	IPwmGenerator *pwm_ = nullptr;

};

#endif /* _ISERVODRIVE_H_ */
