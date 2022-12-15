#include "motorctrlobject.h"

namespace ryno
{

static const char* g_tp_name = "motorctrl";

const char* MotorctrlObject::GetTypeName()
{
	return g_tp_name;
}

RyPointer GetMembers()
{
	static RyPointer members;
	if (members.get())
		return members;	

	members = RyPointer(new MapObject());
	members->AssignAt(
		RyPointer(new StringObject("__dir__")), 
		RyPointer(new SwiObject([](rycpu_t *cpu, const asmins_t *ins){
			rycpu_param_return(cpu, members->Dir());
		})));
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
		RyPointer(new StringObject("pid_current_kp")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.pid_current_kp_));
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.pid_current_kp_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("pid_current_ki")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.pid_current_ki_)); 
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.pid_current_ki_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("pid_current_maxout")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.pid_current_maxout_)); 
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.pid_current_maxout_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("pid_w_kp")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.pid_w_kp_));
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.pid_w_kp_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("pid_w_ki")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.pid_w_ki_)); 
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.pid_w_ki_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("pid_w_maxout")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.pid_w_maxout_)); 
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.pid_w_maxout_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("pid_p_kp")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.pid_p_kp_));
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.pid_p_kp_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("pid_p_maxout")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.pid_p_maxout_)); 
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.pid_p_maxout_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("tau_ratio")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.tau_ratio_)); 
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.tau_ratio_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("vab_advance_factor")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.vab_advance_factor_)); 
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.vab_advance_factor_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("vq_bias")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.vq_bias_)); 
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.vq_bias_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("w_bias")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.w_bias_)); 
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.w_bias_ = v->Float();
			}, 
			nullptr)
		));

	members->AssignAt(
		RyPointer(new StringObject("display")), 
		RyPointer(new PropertyObject(
			[](rycpu_t *cpu, RyPointer ctx)->RyPointer { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				return RyPointer(new FloatObject(value.config_.display_)); 
			}, 
			[](rycpu_t *cpu, RyPointer ctx, const RyPointer& v)->void { 
				MotorCtrlFOC& value = RyObject_GetMotorctrl(cpu->register_get(TH).get());
				value.config_.display_ = v->Float();
			}, 
			nullptr)
		));

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
	ss << "<MotorctrlObject: " << value_.GetAxisId() << "> ";
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

RyPointer* MotorctrlObject::PtrAt(const RyPointer& v)
{
	return GetMembers()->PtrAt(v);
}

} // namespace ryno
