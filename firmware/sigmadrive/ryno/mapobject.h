#ifndef _MAPOBJECT_H_
#define _MAPOBJECT_H_

//#include <map>
#include <unordered_map>
#include <iostream>
#include "ryobject.h"
#include "boolobject.h"
#include "smart_ptr.h"



template<>
struct std::hash<ryno::RyPointer> : public std::__hash_base<size_t, ryno::RyPointer>
{
	size_t operator()(const ryno::RyPointer& __p) const noexcept
	{
		size_t hash = __p->Hash();
		return hash;
	}
};


namespace ryno {


template <typename T>
struct Equal
{
	constexpr bool operator()(const RyPointer& lhs, const RyPointer& rhs) const
	{
		if (lhs->GetType() != rhs->GetType())
			return false;
		return RyObject_GetBool(RyPointer(lhs->Equal(lhs.get(), rhs.get())));
	}
};


#if !defined(RY_MAP_TYPE)
#define RY_MAP_TYPE std::unordered_map<RyPointer, RyPointer, std::hash<RyPointer>, Equal<RyPointer>>
#endif


using RyMap = RY_MAP_TYPE;

inline bool CheckType_Map(RyObject* v);

struct MapObject : RyObject {
	RyMap value_;
	MapObject() = default;
	MapObject(const MapObject& map) : RyObject(), value_(map.value_) {}
	virtual ~MapObject() override {
		// std::cout << "Destroying Map: " << (void*) this << std::endl;
	}
	virtual RyObject* Clone() const override;
	virtual RyObject* Boolean() const override;	
	virtual std::string Repr() const override;
	virtual std::string ToStr() const override;
	virtual RyObject* At(RyObject *v) override;
	virtual RyObject* AssignAt(RyObject *v, RyObject *w) override;
	virtual size_t Length() const override;

	virtual const char* GetType() const override;
	static const char* GetTypeName();
	RyObject* AssignAtPP(RyObject *v, RyObject *w);
};

inline bool CheckType_Map(RyObject* v) { return (v->GetType() == MapObject::GetTypeName()) ? true : false; }
inline RyMap& RyObject_GetMap(RyObject* v)
{
	if (!CheckType_Map(v))
		throw InvalidTypeError(std::string("The type must be 'map', not ") + v->GetType());
	return static_cast<MapObject*>(v)->value_;
}

inline RyMap& RyObject_GetMap(const RyPointer& v)
{
	return RyObject_GetMap(v.get());
}

}

#endif
