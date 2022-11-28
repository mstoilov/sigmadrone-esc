#include "error.h"
#include "mapobject.h"
#include "stringobject.h"
#include "boolobject.h"

namespace ryno {

static const char* g_tp_name = "array";

const char* MapObject::GetTypeName()
{
	return g_tp_name;
}

RyObject* MapObject::Clone() const
{
	MapObject* ret = new MapObject();
	for (RyMap::const_iterator it = value_.begin(); it != value_.end(); it++) {
		ret->value_[it->first] = RyPointer(it->second.get()->Clone());
	}
	return ret;
}

RyObject* MapObject::Boolean() const
{
	return new BoolObject(!static_cast<const MapObject*>(this)->value_.empty());
}

std::string MapObject::Repr() const
{
	std::stringstream ss;
	ss << "{";
	for (RyMap::const_iterator it = value_.begin(); it != value_.end(); it++) {
		if (it != value_.begin()) {
			ss << ", ";
		}
		ss << it->first->Repr() << ": " << it->second->Repr();
	}
	ss << "}";
	return ss.str();
}

std::string MapObject::ToStr() const
{
	return Repr();
}


const char* MapObject::GetType() const
{
	return GetTypeName();
}

RyObject* MapObject::AssignAt(RyObject *v, RyObject *w)
{
	value_[RyPointer(v->Clone())] = RyPointer(w->Clone());
	return w;
}

RyObject* MapObject::AssignAtPP(RyObject *v, RyObject *w)
{
	value_[RyPointer(v)] = RyPointer(w);
	return w;
}

RyObject* MapObject::At(RyObject *v)
{
	RyPointer keyref(v, true);
	RyMap::const_iterator it = value_.find(keyref);
	if (it == value_.end()) {
		throw OutOfRangeError(std::string("Cannot find in map the specified value of: ") + keyref->Repr());
	}
	return it->second.get();
}


std::size_t MapObject::Length() const
{
	return value_.size();
}

}

