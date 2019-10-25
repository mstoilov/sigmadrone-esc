/*
 * servo_drive.h
 *
 *  Created on: Oct 15, 2019
 *      Author: mstoilov
 */

#ifndef _SERVO_DRIVE_H_
#define _SERVO_DRIVE_H_

#include "iservodrive.h"
#include "torque_loop.h"
#include "rpcproperty.h"


class ServoDrive : public IServoDrive {
public:
	ServoDrive();
	virtual ~ServoDrive();
	virtual void Start() override;
	virtual void Stop() override;
	virtual bool IsStarted() override;

	void PeriodElapsedCallback();

public:
	RpcProperty props_;

protected:
	uint32_t update_counter_ = 0;
	uint32_t period_ = 0;
	TorqueLoop tql_;
	uint32_t timings_[3];
};

#endif /* _SERVO_DRIVE_H_ */
