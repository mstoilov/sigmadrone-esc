#ifndef _MOTORCTRL_FOCOBJECT_H_
#define _MOTORCTRL_FOCOBJECT_H_

#include <iostream>
#include <string>
#include "motorctrl_foc.h"
#include "ryno/ryno.h"


namespace ryno {

struct MotorctrlObject : RyObject {
	MotorCtrlFOC value_;
	RyPointer members_;
	MotorctrlObject(MotorDrive* drive, std::string axis_id);
	virtual ~MotorctrlObject() override = default;
	virtual RyPointer Clone() const override;
	virtual std::string Repr() const override;
	virtual std::string Str() const override;
	virtual RyPointer At(const RyPointer& v) override;
	virtual const char* GetType() const override;

};


}

#endif
