#include <cmath>

#include "boolobject.h"
#include "floatobject.h"
#include "intobject.h"
#include "error.h"

namespace ryno {

static const char* g_tp_name = "float";

const char* FloatObject::GetTypeName()
{
	return g_tp_name;
}

const char* FloatObject::GetType() const
{
	return GetTypeName();
}

template<typename OP, typename RET>
RyObject* BinOpCommon(RyObject* v, RyObject* w, const OP& op)
{
	RyFloat a, b;
	if (CheckType_Float(v))
		a = static_cast<FloatObject*>(v)->value_;
	else if (CheckType_Int(v))
		a = static_cast<IntObject*>(v)->value_;
	else
		return nullptr;

	if (CheckType_Float(w))
		b = static_cast<FloatObject*>(w)->value_;
	else if (CheckType_Int(w))
		b = static_cast<IntObject*>(w)->value_;
	else
		return nullptr;
	return new RET(op(a, b));
}

template<typename OP>
RyObject* FloatBinOp(RyObject* v, RyObject* w, const OP& op)
{
	return BinOpCommon<OP, FloatObject>(v, w, op);
}

template<typename OP>
RyObject* BoolBinOp(RyObject* v, RyObject* w, const OP& op)
{
	return BinOpCommon<OP, BoolObject>(v, w, op);
}


FloatObject::FloatObject(RyFloat value) : RyObject(), value_(value)
{
}


RyObject* FloatObject::Clone() const
{
	return new FloatObject(static_cast<const FloatObject*>(this)->value_);
}

RyObject* FloatObject::Boolean() const
{
	return new BoolObject(static_cast<const FloatObject*>(this)->value_);
}

RyObject* FloatObject::Negate() const
{
	return new FloatObject(- static_cast<const FloatObject*>(this)->value_);
}

std::string FloatObject::Repr() const
{
	std::stringstream ss;
	ss << static_cast<const FloatObject*>(this)->value_;
	return ss.str();
}

std::string FloatObject::ToStr() const
{
	return Repr();
}


RyObject* FloatObject::Add(RyObject *v, RyObject *w) const
{
	return FloatBinOp(v, w, std::plus<RyFloat>());
}

RyObject* FloatObject::Subtract(RyObject *v, RyObject *w) const
{
	return FloatBinOp(v, w, std::minus<RyFloat>());
}

RyObject* FloatObject::Multiply(RyObject *v, RyObject *w) const
{
	return FloatBinOp(v, w, std::multiplies<RyFloat>());
}

RyObject* FloatObject::Divide(RyObject *v, RyObject *w) const
{
	if (!static_cast<const IntObject*>(w)->value_)
		throw ZeroDivisionError("Float division by zero");

	return FloatBinOp(v, w, std::divides<RyFloat>());
}

RyObject* FloatObject::Modulus(RyObject *v, RyObject *w) const
{
	return FloatBinOp(v, w, [](RyFloat a, RyFloat b)->RyFloat{return fmod(a, b);});
}

RyObject* FloatObject::Equal(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::equal_to<RyFloat>());
}

RyObject* FloatObject::NotEqual(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::not_equal_to<RyFloat>());
}

RyObject* FloatObject::Greater(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::greater<RyFloat>());
}

RyObject* FloatObject::Less(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::less<RyFloat>());
}

RyObject* FloatObject::GreaterEqual(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::greater_equal<RyFloat>());
}

RyObject* FloatObject::LessEqual(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::less_equal<RyFloat>());
}

std::size_t FloatObject::Hash() const
{
	return std::hash<decltype(value_)>{}(value_);
}

void FloatObject::Inc()
{
	static_cast<FloatObject*>(this)->value_++;
}

void FloatObject::Dec()
{
	static_cast<FloatObject*>(this)->value_--;
}


}
