#ifndef _FUNCTIONOBJECT_H_
#define _FUNCTIONOBJECT_H_

#include <iostream>
#include "codefragment.h"
#include "ryobject.h"
#include "smart_ptr.h"


namespace ryno {

inline bool CheckType_Function(RyObject* v);

struct FunctionObject : RyObject {
	CodeFragment value_;
	FunctionObject() = default;
	FunctionObject(const FunctionObject& ) = default;
	virtual ~FunctionObject() = default;

	FunctionObject(const CodeFragment& value);

	virtual RyObject* Clone() const override;
	virtual RyObject* Boolean() const override;
	virtual std::string Repr() const override;

	virtual const char* GetType() const override;
	static const char* GetTypeName();
};

inline bool CheckType_Function(RyObject* v) { return (v->GetType() == FunctionObject::GetTypeName()) ? true : false; }
inline CodeFragment& RyObject_GetFunction(RyObject* v)
{
	if (!CheckType_Function(v))
		throw InvalidTypeError(std::string("The type must be 'Function', not ") + v->GetType());
	return static_cast<FunctionObject*>(v)->value_;
}

inline CodeFragment& RyObject_GetFunction(const RyPointer& v)
{
	return RyObject_GetFunction(v.get());
}

}

#endif
