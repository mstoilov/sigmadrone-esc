#ifndef _FLOATOBJECT_H_
#define _FLOATOBJECT_H_

#include <iostream>
#include "ryobject.h"
#include "smart_ptr.h"

#if !defined(RY_FLOAT_TYPE)
#define RY_FLOAT_TYPE double
#endif

namespace ryno {

using RyFloat = RY_FLOAT_TYPE;

inline bool CheckType_Float(RyObject* v);

struct FloatObject : RyObject {
	RyFloat value_;
	FloatObject(RyFloat value);
//	virtual ~FloatObject() override = default;
	~FloatObject()
	{
//		std::cout << "Destroying(float): " << Repr() << std::endl;
	}

	virtual RyObject* Clone() const override;
	virtual RyObject* Boolean() const override;	
	virtual RyObject* Negate() const override;
	virtual std::string Repr() const override;
	virtual std::string ToStr() const override;
	virtual RyObject* Subtract(RyObject *, RyObject *) const override;
	virtual RyObject* Multiply(RyObject *, RyObject *) const override;
	virtual RyObject* Divide(RyObject *, RyObject *) const override;
	virtual RyObject* Modulus(RyObject *, RyObject *) const override;
	virtual RyObject* Equal(RyObject *, RyObject *) const override;
	virtual RyObject* NotEqual(RyObject *, RyObject *) const override;
	virtual RyObject* Add(RyObject *, RyObject *) const override;
	virtual RyObject* Greater(RyObject *, RyObject *) const override;
	virtual RyObject* Less(RyObject *, RyObject *) const override;
	virtual RyObject* GreaterEqual(RyObject *, RyObject *) const override;
	virtual RyObject* LessEqual(RyObject *, RyObject *) const override;
	virtual size_t Hash() const override;
	virtual void Inc() override;
	virtual void Dec() override;
	virtual const char* GetType() const override;

	static const char* GetTypeName();
};

inline bool CheckType_Float(RyObject* v) { return (v->GetType() == FloatObject::GetTypeName()) ? true : false; }
inline RyFloat& RyObject_GetFloat(RyObject* v)
{
	if (!CheckType_Float(v))
		throw InvalidTypeError(std::string("The type must be 'float', not ") + v->GetType());
	return static_cast<FloatObject*>(v)->value_;
}

inline RyFloat& RyObject_GetFloat(const RyPointer& v)
{
	return RyObject_GetFloat(v.get());
}

}

#endif
