#include "motorctrlobject.h"

namespace ryno
{

static const char* g_tp_name = "motorctrl";

const char* MotorctrlObject::GetTypeName()
{
	return g_tp_name;
}

static RyPointer GetMembers()
{
	static RyPointer members;
	if (members.get())
		return members;	
	members = RyPointer(new MapObject());
	members->AssignAt(
		RyPointer(new StringObject("stop")), 
		RyPointer(new SwiObject([](rycpu_t *cpu, const asmins_t *ins){
			MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
			value.Stop();
			rycpu_param_return(cpu, GetNilObject() );
		})));
	members->AssignAt(
		RyPointer(new StringObject("modeclps")), 
		RyPointer(new SwiObject([](rycpu_t *cpu, const asmins_t *ins){
			MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
			value.ModeClosedLoopPositionSimple();
			rycpu_param_return(cpu, GetNilObject() );
		})));
	members->AssignAt(
		RyPointer(new StringObject("modeclpt")), 
		RyPointer(new SwiObject([](rycpu_t *cpu, const asmins_t *ins){
			MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
			value.ModeClosedLoopPositionTrajectory();
			rycpu_param_return(cpu, GetNilObject() );
		})));
	members->AssignAt(
		RyPointer(new StringObject("modeclv")), 
		RyPointer(new SwiObject([](rycpu_t *cpu, const asmins_t *ins){
			MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
			value.ModeClosedLoopVelocity();
			rycpu_param_return(cpu, GetNilObject() );
		})));
	members->AssignAt(
		RyPointer(new StringObject("mv")), 
		RyPointer(new SwiObject([](rycpu_t *cpu, const asmins_t *ins){
			rycpu_stackparam_assert_min_count(cpu, 1);
			size_t position = rycpu_stackparam_get(cpu, 0)->Int();
			MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
			value.MoveToPosition(position);
			rycpu_param_return(cpu, GetNilObject() );
		})));
	members->AssignAt(
		RyPointer(new StringObject("rps")), 
		RyPointer(new PropertyObject([](rycpu_t *cpu, RyPointer ctx)->RyPointer { return RyPointer(new IntObject(100)); }, 
			nullptr, nullptr)
		));
	return members;
}

const char* MotorctrlObject::GetType() const
{
	return g_tp_name;
}

MotorctrlObject::MotorctrlObject(MotorDrive* drive, std::string axis_id) : value_(drive, axis_id)
{
}

RyPointer MotorctrlObject::Clone() const
{
	return const_cast<RyObject*>((RyObject*)this)->shared_from_this();
}

std::string MotorctrlObject::Repr() const
{
	std::stringstream ss;
	ss << "<MotorctrlObject: " << value_.GetAxisId() << ">";
	return ss.str();
}

std::string MotorctrlObject::Str() const
{
	return Repr();
}

RyPointer MotorctrlObject::At(const RyPointer& v)
{
	return GetMembers()->At(v);
}

} // namespace ryno
