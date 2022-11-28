#ifndef _RYOBJECT_H_
#define _RYOBJECT_H_

#include <vector>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <variant>
#include <cstring>
#include <sstream>

#include "error.h"
#include "smart_ptr.h"

#if !defined(RY_INTEGER_TYPE)
#define RY_INTEGER_TYPE int64_t
#endif


namespace ryno {

using RyInt = RY_INTEGER_TYPE;


struct RyObject {
	RyObject() = default;
	virtual ~RyObject() = default;

	virtual RyObject* Clone() const		{ ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* Boolean() const	{ ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }	
	virtual RyObject* Negate() const	{ ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual std::string Repr() const 	{ std::stringstream ss; ss << "<" << (void*)this << ">"; return ss.str(); }
	virtual std::string ToStr() const	{ ThrowOperationNotImplemented(__FUNCTION__); return ""; }
	virtual RyObject* Subtract(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* Multiply(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* Divide(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* Modulus(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* LeftShift(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* RightShift(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* Append(RyObject *v) { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; };
	virtual RyObject* At(RyObject *v) { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; };
	virtual RyObject* At(RyInt index) { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; };
	virtual RyObject* AssignAt(RyObject *v, RyObject *w) { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; };
	virtual RyObject* AssignAt(RyInt index, RyObject *w) { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; };
	virtual size_t Length() const { ThrowOperationNotImplemented(__FUNCTION__); return 0; };
	virtual RyObject* Equal(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* NotEqual(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* Add(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* Greater(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* Less(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* GreaterEqual(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual RyObject* LessEqual(RyObject *, RyObject *) const { ThrowOperationNotImplemented(__FUNCTION__); return nullptr; }
	virtual size_t Hash() const { ThrowOperationNotImplemented(__FUNCTION__); return 0; }
	virtual void Inc() { ThrowOperationNotImplemented(__FUNCTION__); }
	virtual void Dec() { ThrowOperationNotImplemented(__FUNCTION__); }

	virtual const char* GetType() const { return nullptr; }
};


typedef smart_ptr<RyObject> RyPointer;


}


#endif // _RYOBJECT_H_
