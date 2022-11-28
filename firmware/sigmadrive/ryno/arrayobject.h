#ifndef _ARRAYOBJECT_H_
#define _ARRAYOBJECT_H_

#include <vector>
#include "smart_ptr.h"
#include "ryobject.h"


#if !defined(RY_ARRAY_TYPE)
#define RY_ARRAY_TYPE std::vector<RyPointer>
#endif

namespace ryno {

using RyArray = RY_ARRAY_TYPE;

inline bool CheckType_Array(RyObject* v);

struct ArrayObject : RyObject {
	std::vector<RyPointer> value_;
	ArrayObject() = default;
	virtual ~ArrayObject() override = default;
	virtual RyObject* Clone() const override;
	virtual RyObject* Boolean() const override;	
	virtual std::string Repr() const override;
	virtual std::string ToStr() const override;
	virtual RyObject* Append(RyObject *v) override;
	virtual RyObject* At(RyObject *v) override;
	virtual RyObject* At(RyInt index) override;
	virtual RyObject* AssignAt(RyObject *v, RyObject *w) override;
	virtual RyObject* AssignAt(RyInt index, RyObject *w) override;
	virtual size_t Length() const override;

	size_t emplace_back(RyObject* v);

	virtual const char* GetType() const override;
	static const char* GetTypeName();

};

inline bool CheckType_Array(RyObject* v) { return (v->GetType() == ArrayObject::GetTypeName()) ? true : false; }
inline RyArray& RyObject_GetArray(RyObject* v)
{
	if (!CheckType_Array(v))
		throw InvalidTypeError(std::string("The type must be 'array', not ") + v->GetType());
	return static_cast<ArrayObject*>(v)->value_;
}

inline RyArray& RyObject_GetArray(const RyPointer& v)
{
	return RyObject_GetArray(v.get());
}
}

#endif
