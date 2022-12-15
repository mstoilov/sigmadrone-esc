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
	virtual RyPointer* PtrAt(const RyPointer& v) override;
	virtual const char* GetType() const override;

	static const char* GetTypeName();
};

inline bool CheckType_Motorctrl(const RyObject* v)
{
	return (v->GetType() == MotorctrlObject::GetTypeName()) ? true : false; 
}

inline bool CheckType_Motorctrl(const RyPointer& v) 
{
	return CheckType_Motorctrl(v.get());
}

inline const MotorCtrlFOC& RyObject_GetMotorctrl(const RyObject* v)
{
	if (!v) 
		ThrowInvalidParameter();
	if (!CheckType_Motorctrl(v))
		throw InvalidTypeError(std::string("The type must be 'motorctrl', not ") + v->GetType());
	return static_cast<const MotorctrlObject*>(v)->value_;
}

inline MotorCtrlFOC& RyObject_GetMotorctrl(RyObject* v)
{
	if (!v) 
		ThrowInvalidParameter();
	if (!CheckType_Motorctrl(v))
		throw InvalidTypeError(std::string("The type must be 'motorctrl', not ") + v->GetType());
	return static_cast<MotorctrlObject*>(v)->value_;
}

}

#endif
