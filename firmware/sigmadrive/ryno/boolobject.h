#ifndef _BOOLOBJECT_H_
#define _BOOLOBJECT_H_

#include <iostream>
#include "ryobject.h"
#include "smart_ptr.h"


namespace ryno {

inline bool CheckType_Bool(RyObject* v);

struct BoolObject : RyObject {
	bool value_;
	BoolObject(bool value);
	BoolObject(const std::string& value);
	virtual ~BoolObject()
	{
	}
	virtual RyObject* Clone() const override;
	virtual RyObject* Boolean() const override;	
	virtual std::string Repr() const override;
	virtual std::string ToStr() const override;
	virtual std::size_t Hash() const override;
	virtual RyObject* Equal(RyObject *, RyObject *) const override;
	virtual RyObject* NotEqual(RyObject *, RyObject *) const override;
	virtual RyObject* Greater(RyObject *, RyObject *) const override;
	virtual RyObject* Less(RyObject *, RyObject *) const override;
	virtual RyObject* GreaterEqual(RyObject *, RyObject *) const override;
	virtual RyObject* LessEqual(RyObject *, RyObject *) const override;

	virtual const char* GetType() const override;
	static const char* GetTypeName();
};

inline bool CheckType_Bool(RyObject* v) { return (v->GetType() == BoolObject::GetTypeName()) ? true : false; }
inline bool& RyObject_GetBool(RyObject* v)
{
	if (!CheckType_Bool(v))
		throw InvalidTypeError(std::string("The type must be 'Bool', not ") + v->GetType());
	return static_cast<BoolObject*>(v)->value_;
}

inline bool& RyObject_GetBool(const RyPointer& v)
{
	return RyObject_GetBool(v.get());
}

}

#endif
