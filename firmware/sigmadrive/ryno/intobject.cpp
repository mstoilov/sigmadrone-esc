
#include "boolobject.h"
#include "floatobject.h"
#include "intobject.h"
#include "error.h"

namespace ryno {

static const char* g_tp_name = "int";

template<typename OP, typename RET>
RyObject* BinOpCommon(RyObject* v, RyObject* w, const OP& op)
{
	if (v->GetType() != w->GetType() || !CheckType_Int(w)) {
		return nullptr;
	}
	return new RET(op(static_cast<const IntObject*>(v)->value_, static_cast<const IntObject*>(w)->value_));
}

template<typename OP>
RyObject* IntBinOp(RyObject* v, RyObject* w, const OP& op)
{
	return BinOpCommon<OP, IntObject>(v, w, op);
}

template<typename OP>
RyObject* BoolBinOp(RyObject* v, RyObject* w, const OP& op)
{
	return BinOpCommon<OP, BoolObject>(v, w, op);
}

const char* IntObject::GetTypeName()
{
	return g_tp_name;
}

const char* IntObject::GetType() const
{
	return GetTypeName();
}

IntObject::IntObject(RyInt value) : RyObject(), value_(value)
{
}

IntObject::IntObject(const char* begin, const char *end)
{

}

RyObject* IntObject::Clone() const
{
	return new IntObject(static_cast<const IntObject*>(this)->value_);
}

RyObject* IntObject::Boolean() const
{
	return new BoolObject(static_cast<const IntObject*>(this)->value_);
}

RyObject* IntObject::Negate() const
{
	return new IntObject(- static_cast<const IntObject*>(this)->value_);
}


std::string IntObject::Repr() const
{
	std::stringstream ss;
	ss << static_cast<const IntObject*>(this)->value_;
	return ss.str();
}

std::string IntObject::ToStr() const
{
	return Repr();
}


RyObject* IntObject::Equal(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::equal_to<RyInt>());
}

RyObject* IntObject::NotEqual(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::not_equal_to<RyInt>());
}

RyObject* IntObject::Greater(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::greater<RyInt>());
}

RyObject* IntObject::Less(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::less<RyInt>());
}

RyObject* IntObject::GreaterEqual(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::greater_equal<RyInt>());
}

RyObject* IntObject::LessEqual(RyObject *v, RyObject *w) const
{
	return BoolBinOp(v, w, std::less_equal<RyInt>());
}

RyObject* IntObject::Add(RyObject *v, RyObject *w) const
{
	return IntBinOp(v, w, std::plus<RyInt>());
}

RyObject* IntObject::Subtract(RyObject *v, RyObject *w) const
{
	return IntBinOp(v, w, std::minus<RyInt>());
}

RyObject* IntObject::Multiply(RyObject *v, RyObject *w) const
{
	return IntBinOp(v, w, std::multiplies<RyInt>());
}

RyObject* IntObject::Divide(RyObject *v, RyObject *w) const
{
	if (!static_cast<const IntObject*>(w)->value_)
		throw ZeroDivisionError("Division by zero");
	return IntBinOp(v, w, std::divides<RyInt>());
}

RyObject* IntObject::Modulus(RyObject *v, RyObject *w) const
{
	return IntBinOp(v, w, [](RyInt a, RyInt b)->RyInt{return a % b;});
}

RyObject* IntObject::LeftShift(RyObject *v, RyObject *w) const
{
	return IntBinOp(v, w, [](RyInt a, RyInt b)->RyInt{return a << b;});
}

RyObject* IntObject::RightShift(RyObject *v, RyObject *w) const
{
	return IntBinOp(v, w, [](RyInt a, RyInt b)->RyInt{return a >> b;});
}

std::size_t IntObject::Hash() const
{
	return std::hash<decltype(value_)>{}(value_);
}

void IntObject::Inc()
{
	static_cast<IntObject*>(this)->value_++;
}

void IntObject::Dec()
{
	static_cast<IntObject*>(this)->value_--;
}


}
