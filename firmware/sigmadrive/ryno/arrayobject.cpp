#include "error.h"
#include "arrayobject.h"
#include "intobject.h"
#include "boolobject.h"

namespace ryno {

static const char* g_tp_name = "array";

const char* ArrayObject::GetTypeName()
{
	return g_tp_name;
}

RyObject* ArrayObject::Clone() const
{
	ArrayObject* ret = new ArrayObject();
	for (RyArray::const_iterator it = value_.begin(); it < value_.end(); it++) {
		ret->value_.emplace_back((*it)->Clone());
	}
	return ret;
}

RyObject* ArrayObject::Boolean() const
{
	return new BoolObject(!static_cast<const ArrayObject*>(this)->value_.empty());
}


std::string ArrayObject::Repr() const
{
	std::stringstream ss;
	ss << "[";
	for (RyArray::const_iterator it = value_.begin(); it < value_.end(); it++) {
		if (it != value_.begin()) {
			ss << ", ";
		}
		ss << (*it)->Repr();
	}
	ss << "]";
	return ss.str();
}

std::string ArrayObject::ToStr() const
{
	return Repr();
}

const char* ArrayObject::GetType() const
{
	return GetTypeName();
}

RyObject* ArrayObject::Append(RyObject *v)
{
	value_.emplace_back(v);
	return value_.rbegin()->get();
}

RyObject* ArrayObject::At(RyObject *v)
{
	RyInt index = RyObject_GetInt(v);
	return At(index);
}


RyObject* ArrayObject::At(RyInt index)
{
	if (std::abs(index) >= (RyInt)value_.size())
		throw OutOfRangeError("Invalid index value, it cannot be larger than the array size.");
	if (index >= 0) {
		return value_[index].get();
	} else {
		RyArray::reverse_iterator it = value_.rbegin();
		it += std::abs(index);
		return (*it).get();
	}
}


RyObject* ArrayObject::AssignAt(RyObject *v, RyObject *w)
{
	RyInt index = RyObject_GetInt(v);
	return AssignAt(index, w);
}

RyObject* ArrayObject::AssignAt(RyInt index, RyObject *w)
{
	if (value_.size() == 0)
		throw OutOfRangeError("array index out of range");
	if (index < 0)
		index = value_.size() - std::abs(index);
	if (index < 0)
		throw OutOfRangeError("array index out of range");
	value_.at(index) = RyPointer(w->Clone());
	return value_.at(index).get();
}

std::size_t ArrayObject::Length() const
{
	return value_.size();
}

size_t ArrayObject::emplace_back(RyObject* v)
{
	size_t idx = Length();
	value_.emplace_back(v);
	return idx;
}

}

