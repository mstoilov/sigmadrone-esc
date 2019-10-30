/*
 * rpcproperty.h
 *
 *  Created on: Oct 22, 2019
 *      Author: mstoilov
 */

#ifndef RPCPROPERTY_H_
#define RPCPROPERTY_H_

#include <functional>
#include "rexjson++.h"

class RpcProperty;

typedef enum {readonly = 1, writeonly = 2, readwrite = 3} RpcObjectAccess;

class IRpcPropertyObject {
public:
	using ModifiedHook = void (*)(void*);

	virtual ~IRpcPropertyObject() { }
	virtual IRpcPropertyObject* duplicate() = 0;
	virtual void SetProp(const rexjson::value& val) = 0;
	virtual rexjson::value GetProp() = 0;
};


template<typename PropType>
class RpcPropertyObject : public IRpcPropertyObject {
public:
	RpcPropertyObject(
			PropType* propaddr,
			RpcObjectAccess access,
			const std::function<void(const rexjson::value&)>& check_hook,
			const std::function<void(void *ctx)>& modified_hook,
			void *ctx)
	: propaddr_(propaddr)
	, access_(access)
	, check_hook_(check_hook)
	, modified_hook_(modified_hook)
	, ctx_(ctx)
	{}
	PropType* propaddr_;
	RpcObjectAccess access_ = readwrite;
	std::function<void(const rexjson::value&)> check_hook_ = {};
	std::function<void(void *ctx)> modified_hook_ = {};
	void *ctx_ = nullptr;

	virtual IRpcPropertyObject* duplicate()
	{
		return new RpcPropertyObject(propaddr_, access_, check_hook_, modified_hook_, ctx_);
	}

	virtual void SetProp(const rexjson::value& val) override
	{
		if ((access_ & RpcObjectAccess::writeonly) == 0)
			throw std::runtime_error("No write access");
		check_hook_(val);
		SetPropImpl<PropType>(propaddr_, val);
		modified_hook_(ctx_);
	}

	virtual rexjson::value GetProp() override
	{
		if ((access_ & RpcObjectAccess::readonly) == 0)
			throw std::runtime_error("No read access");
		rexjson::value val = GetPropImpl(propaddr_);
		return val;
	}

	template <typename U>
	typename std::enable_if<std::is_integral<U>::value || std::is_enum<U>::value, int64_t>::type
	GetPropImpl(U* prop)
	{
		return *prop;
	}

	template <typename U>
	typename std::enable_if<std::is_floating_point<U>::value, double>::type
	GetPropImpl(U* prop)
	{
		return *prop;
	}


	template <typename U>
	typename std::enable_if<std::is_base_of<std::string, U>::value, std::string>::type
	GetPropImpl(U* prop)
	{
		return *prop;
	}

	template <typename U>
	typename std::enable_if<std::is_integral<U>::value, void>::type
	SetPropImpl(U* prop, const rexjson::value& val)
	{
		if (val.get_type() == rexjson::int_type) {
			*prop = val.get_int();
		}
	}

	template <typename U>
	typename std::enable_if<std::is_floating_point<U>::value, void>::type
	SetPropImpl(U* prop, const rexjson::value& val)
	{
		*prop = val.get_real();
	}

	template <typename U>
	typename std::enable_if<std::is_base_of<std::string, U>::value, void>::type
	SetPropImpl(U* prop, const rexjson::value& val)
	{
		*prop = val.get_str();
	}

	template <typename U>
	typename std::enable_if<std::is_enum<U>::value, void>::type
	SetPropImpl(U* prop, const rexjson::value& val)
	{
		*reinterpret_cast<typename std::underlying_type<U>::type*>(prop) = val.get_int();
	}
};

template<>
class RpcPropertyObject<bool> : public IRpcPropertyObject {
public:
	RpcPropertyObject(
			bool* propaddr,
			RpcObjectAccess access = RpcObjectAccess::readwrite,
			const std::function<void(const rexjson::value&)>& check_hook = {},
			const std::function<void(void *ctx)>& modified_hook = {},
			void *ctx = nullptr)
	: propaddr_(propaddr)
	, access_(access)
	, check_hook_(check_hook)
	, modified_hook_(modified_hook)
	, ctx_(ctx)
	{}
	bool* propaddr_;
	RpcObjectAccess access_ = readwrite;
	std::function<void(const rexjson::value&)> check_hook_ = {};
	std::function<void(void *ctx)> modified_hook_ = {};
	void *ctx_ = nullptr;


	virtual IRpcPropertyObject* duplicate()
	{
		return new RpcPropertyObject(propaddr_, access_, check_hook_, modified_hook_, ctx_);
	}

	virtual void SetProp(const rexjson::value& val) override
	{
		if ((access_ & RpcObjectAccess::writeonly) == 0)
			throw std::runtime_error("No write access");
		check_hook_(val);
		*propaddr_ = val.get_bool();
		modified_hook_(ctx_);
	}

	virtual rexjson::value GetProp() override
	{
		if ((access_ & RpcObjectAccess::readonly) == 0)
			throw std::runtime_error("No read access");
		return *propaddr_;
	}
};


using RpcPropertyMap = std::map<std::string, RpcProperty>;
using RpcPropertyArray = std::vector<RpcProperty>;
using RpcPropertyCallback = std::function<void(const std::string&, RpcProperty&)>;

class RpcProperty {
public:
	using Type = enum {null_type = 0, map_type, array_type, object_type};

	virtual ~RpcProperty();
	template<typename T>
	RpcProperty(
			T* propaddr,
			RpcObjectAccess access = RpcObjectAccess::readwrite,
			const std::function<void(const rexjson::value&)>& check_hook = [](const rexjson::value& v)->void{},
			const std::function<void(void *ctx)>& modified_hook = [](void *ctx)->void{},
			void *ctx = nullptr)
	{
		object_ = static_cast<IRpcPropertyObject*>(new RpcPropertyObject<T>(propaddr, access, check_hook, modified_hook, ctx));
		type_ = object_type;
	}
	RpcProperty();
	RpcProperty(const RpcProperty& v);
	RpcProperty(const RpcPropertyMap& map);
	RpcProperty(const RpcPropertyArray& array);
	void check_type(Type vtype) const;
	RpcProperty& push_back(const RpcProperty& v);
	RpcProperty& operator[](size_t i);
	RpcProperty& operator[](const std::string& name);
	RpcProperty& operator=(const RpcProperty& v);
	RpcProperty& Navigate(const std::string& path);
	void SetProp(const rexjson::value& val);
	rexjson::value GetProp();
	void Destroy();
	static std::string GetTypeName(Type type)
	{
		return rpcproperty_type_name[type];
	}

	void EnumChildren(const std::string& path, const RpcPropertyCallback& callback);

protected:
	static const char* rpcproperty_type_name[];
	Type type_ = null_type;

	union {
		RpcPropertyMap *map_;
		RpcPropertyArray *array_;
		IRpcPropertyObject* object_;
	};
};

#endif /* RPCPROPERTY_H_ */
