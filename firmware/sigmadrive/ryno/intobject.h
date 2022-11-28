#ifndef _INTOBJECT_H_
#define _INTOBJECT_H_

#include <iostream>
#include "ryobject.h"
#include "smart_ptr.h"


namespace ryno {

inline bool CheckType_Int(RyObject* v);

struct IntObject : RyObject {
	RyInt value_;
	IntObject(RyInt value);
	IntObject(const char* begin, const char *end);
//	virtual ~IntObject() override = default;
	virtual ~IntObject()
	{
		// std::cout << "Destroying(int): " << Repr() << std::endl;
	}
	virtual RyObject* Clone() const override;
	virtual RyObject* Boolean() const override;
	virtual RyObject* Negate() const override;
	virtual std::string Repr() const override;
	virtual std::string ToStr() const override;
	virtual RyObject* Add(RyObject *, RyObject *) const override;
	virtual RyObject* Subtract(RyObject *, RyObject *) const override;
	virtual RyObject* Multiply(RyObject *, RyObject *) const override;
	virtual RyObject* Divide(RyObject *, RyObject *) const override;
	virtual RyObject* Modulus(RyObject *, RyObject *) const override;
	virtual RyObject* LeftShift(RyObject *, RyObject *) const override;
	virtual RyObject* RightShift(RyObject *, RyObject *) const override;
	virtual std::size_t Hash() const override;
	virtual RyObject* Equal(RyObject *, RyObject *) const override;
	virtual RyObject* NotEqual(RyObject *, RyObject *) const override;
	virtual RyObject* Greater(RyObject *, RyObject *) const override;
	virtual RyObject* Less(RyObject *, RyObject *) const override;
	virtual RyObject* GreaterEqual(RyObject *, RyObject *) const override;
	virtual RyObject* LessEqual(RyObject *, RyObject *) const override;
	virtual void Inc() override;
	virtual void Dec() override;

	virtual const char* GetType() const override;
	static const char* GetTypeName();
};

inline bool CheckType_Int(RyObject* v) { return (v->GetType() == IntObject::GetTypeName()) ? true : false; }
inline RyInt& RyObject_GetInt(RyObject* v)
{
	if (!CheckType_Int(v))
		throw InvalidTypeError(std::string("The type must be 'int', not ") + v->GetType());
	return static_cast<IntObject*>(v)->value_;
}

inline RyInt& RyObject_GetInt(const RyPointer& v)
{
	return RyObject_GetInt(v.get());
}

}

#endif
