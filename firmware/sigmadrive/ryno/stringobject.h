#ifndef _STRINGOBJECT_H_
#define _STRINGOBJECT_H_

#include <string>
#include <iostream>
#include "smart_ptr.h"
#include "ryobject.h"


#if !defined(RY_STRING_TYPE)
#define RY_STRING_TYPE std::string
#endif

namespace ryno {

using RyString = RY_STRING_TYPE;

inline bool CheckType_String(RyObject* v);

struct StringObject : RyObject {
	RyString value_;
	StringObject() = default;
	StringObject(const std::string& str);
//	virtual ~StringObject() override = default;
	virtual ~StringObject() override
	{
//		std::cout << "Destroying (" << this << "): " << Repr() << std::endl;
	}
	virtual RyObject* Clone() const override;
	virtual RyObject* Boolean() const override;	
	virtual RyObject* Equal(RyObject *, RyObject *) const override;
	virtual RyObject* NotEqual(RyObject *, RyObject *) const override;
	virtual std::string Repr() const override;
	virtual std::string ToStr() const override;
	virtual RyObject* Add(RyObject *, RyObject *) const override;
	virtual RyObject* At(RyObject *v) override;
	virtual std::size_t Length() const override;
	virtual std::size_t Hash() const override;

	virtual const char* GetType() const override;
	static const char* GetTypeName();
};

inline bool CheckType_String(RyObject* v) { return (v->GetType() == StringObject::GetTypeName()) ? true : false; }
inline RyString& RyObject_GetString(RyObject* v)
{
	if (!CheckType_String(v))
		throw std::runtime_error(std::string("The type must be 'string', not ") + v->GetType());
	return static_cast<StringObject*>(v)->value_;
}

inline RyString& RyObject_GetString(const RyPointer& v)
{
	return RyObject_GetString(v.get());
}


}

#endif
