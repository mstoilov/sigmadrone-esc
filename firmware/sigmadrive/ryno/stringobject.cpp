#include "stringobject.h"
#include "intobject.h"
#include "boolobject.h"

namespace ryno {

static const char* g_tp_name = "array";

const char* StringObject::GetTypeName()
{
	return g_tp_name;
}

StringObject::StringObject(const std::string& str) : RyObject(), value_(str)
{
}

RyObject* StringObject::Equal(RyObject *v, RyObject *w) const
{
	return new BoolObject(std::equal_to<RyString>{}(RyObject_GetString(v), RyObject_GetString(w)));
}

RyObject* StringObject::NotEqual(RyObject *v, RyObject *w) const
{
	return new BoolObject(std::not_equal_to<RyString>{}(RyObject_GetString(v), RyObject_GetString(w)));
}


RyObject* StringObject::Clone() const
{
	return new StringObject(value_);
}

RyObject* StringObject::Boolean() const
{
	return new BoolObject(!static_cast<const StringObject*>(this)->value_.empty());
}

std::string StringObject::Repr() const
{
	std::stringstream ss;
	ss << "'" << value_ << "'";
	return ss.str();
}

std::string StringObject::ToStr() const
{
	return value_;
}

const char* StringObject::GetType() const
{
	return GetTypeName();
}

RyObject* StringObject::Add(RyObject *v, RyObject *w) const
{
	return new StringObject(RyObject_GetString(v) + RyObject_GetString(w));
}

RyObject* StringObject::At(RyObject *v)
{
	RyInt index = RyObject_GetInt(v);

	if (std::abs(index) >= (RyInt)value_.size())
		throw OutOfRangeError("Invalid index value, it cannot be larger than the array size.");
	if (index >= 0) {
		return new StringObject(RyString(1, value_[index]));
	} else {
		RyString::reverse_iterator it = value_.rbegin();
		it += std::abs(index);
		return new StringObject(RyString(1, *it));
	}
}

std::size_t StringObject::Length() const
{
	return value_.size();
}

std::size_t StringObject::Hash() const
{
	return std::hash<decltype(value_)>{}(value_);
}

}
