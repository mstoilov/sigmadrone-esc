
#include "floatobject.h"
#include "intobject.h"
#include "boolobject.h"
#include "error.h"

namespace ryno {

static const char* g_tp_name = "bool";

template<typename OP>
RyObject* BinOp(RyObject* v, RyObject* w, const OP& op)
{
	if (v->GetType() != w->GetType() || !CheckType_Bool(w)) {
		return nullptr;
	}
	return new BoolObject(op(static_cast<const BoolObject*>(v)->value_, static_cast<const BoolObject*>(w)->value_));
}


const char* BoolObject::GetTypeName()
{
	return g_tp_name;
}

const char* BoolObject::GetType() const
{
	return GetTypeName();
}

BoolObject::BoolObject(bool value) : RyObject(), value_(value)
{
}

BoolObject::BoolObject(const std::string& value)
{
	if (value == "true")
		value_ = true;
	else if (value == "false")
		value_ = false;
	else
		throw InvalidTypeError("BoolObject must be 'true' or 'false', invalid value specified: " + value);
}

RyObject* BoolObject::Clone() const
{
	return new BoolObject(static_cast<const BoolObject*>(this)->value_);
}

RyObject* BoolObject::Boolean() const
{
	return Clone();
}

std::string BoolObject::Repr() const
{
	std::stringstream ss;
	ss << std::boolalpha << static_cast<const BoolObject*>(this)->value_;
	return ss.str();
}

std::string BoolObject::ToStr() const
{
	return Repr();
}


RyObject* BoolObject::Equal(RyObject *v, RyObject *w) const
{
	return BinOp(v, w, std::equal_to<bool>());
}

RyObject* BoolObject::NotEqual(RyObject *v, RyObject *w) const
{
	return BinOp(v, w, std::not_equal_to<bool>());
}

RyObject* BoolObject::Greater(RyObject *v, RyObject *w) const
{
	return BinOp(v, w, std::greater<bool>());
}

RyObject* BoolObject::Less(RyObject *v, RyObject *w) const
{
	return BinOp(v, w, std::less<bool>());
}

RyObject* BoolObject::GreaterEqual(RyObject *v, RyObject *w) const
{
	return BinOp(v, w, std::greater_equal<bool>());
}

RyObject* BoolObject::LessEqual(RyObject *v, RyObject *w) const
{
	return BinOp(v, w, std::less_equal<bool>());
}

std::size_t BoolObject::Hash() const
{
	return std::hash<decltype(value_)>{}(value_);
}

}
