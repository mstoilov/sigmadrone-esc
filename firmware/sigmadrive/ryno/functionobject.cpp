
#include "boolobject.h"
#include "floatobject.h"
#include "functionobject.h"
#include "error.h"

namespace ryno {

static const char* g_tp_name = "CodeFragment";


const char* FunctionObject::GetTypeName()
{
	return g_tp_name;
}

const char* FunctionObject::GetType() const
{
	return GetTypeName();
}

FunctionObject::FunctionObject(const CodeFragment& value) : RyObject(), value_(value)
{
}

RyObject* FunctionObject::Clone() const
{
	return new FunctionObject(value_);
}

RyObject* FunctionObject::Boolean() const
{
	return new BoolObject(!static_cast<const FunctionObject*>(this)->value_.basicblocks_.empty());
}

std::string FunctionObject::Repr() const
{
	std::stringstream ss;
	ss << "<Function: " << static_cast<const FunctionObject*>(this)->value_.name_ << ">";
	return ss.str();
}

}
