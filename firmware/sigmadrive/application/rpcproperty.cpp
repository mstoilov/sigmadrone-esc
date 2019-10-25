/*
 * rpcproperty.cpp
 *
 *  Created on: Oct 22, 2019
 *      Author: mstoilov
 */

#include "rpcproperty.h"

const char* RpcProperty::rpcproperty_type_name[] = {"null", "map", "array", "object"};

RpcProperty::~RpcProperty()
{
	Destroy();
}

RpcProperty::RpcProperty()
{
	type_ = null_type;
	object_ = nullptr;
	map_ = nullptr;
	array_ = nullptr;
}

RpcProperty::RpcProperty(const RpcProperty& v)
{
	operator=(v);
}

RpcProperty::RpcProperty(const RpcPropertyMap& map)
{
	map_ = new RpcPropertyMap(map);
	type_ = map_type;
}

RpcProperty::RpcProperty(const RpcPropertyArray& array)
{
	array_ = new RpcPropertyArray(array);
	type_ = array_type;
}

void RpcProperty::Destroy()
{
	if (type_ == map_type)
		delete map_;
	else if (type_ == array_type)
		delete array_;
	else if (type_ == object_type)
		 delete object_;
	type_= null_type;
}

void RpcProperty::check_type(Type vtype) const
{
	if( type_ != vtype ) {
		std::ostringstream os;
		os << "RpcProperty is type '" << GetTypeName(type_) << "', expected '" << GetTypeName(vtype) << "'";
		throw std::runtime_error( os.str() );
	}
}

RpcProperty& RpcProperty::push_back(const RpcProperty& v)
{
	check_type(array_type);
	array_->push_back(v);
	return *array_->rbegin();
}

RpcProperty& RpcProperty::operator[](size_t i)
{
	check_type(array_type);
	if (i >= array_->size())
		throw std::range_error("RpcPropertyArray: invalid index");
	return array_->operator[](i);
}

RpcProperty& RpcProperty::operator[](const std::string& name)
{
	check_type(map_type);
	return map_->operator[](name);
}

RpcProperty& RpcProperty::operator=(const RpcProperty& v)
{
	if (&v == this)
		return *this;
	Destroy();
	type_ = v.type_;
	if (type_ == array_type)
		array_ = new RpcPropertyArray(*v.array_);
	else if (type_ == map_type)
		map_ =  new RpcPropertyMap(*v.map_);
	else if (type_ == object_type)
		object_ = v.object_->duplicate();
	return *this;
}

RpcProperty& RpcProperty::Navigate(const std::string& path)
{
	if (type_ == object_type) {
		if (!path.empty())
			throw (std::runtime_error("Invalid path"));
		return *this;
	} else if (type_ == map_type) {
		std::string toc = path;
		std::size_t toc_pos = toc.find_first_of(".[", 1);
		if (toc_pos != std::string::npos)
			toc = toc.substr(0, toc_pos);
		else
			toc = toc.substr(0);
		if (toc.empty())
			throw (std::runtime_error("Invalid path"));
		std::string restpath = path.substr(toc.size());
		std::string trimmed = (toc.size() && toc.at(0) == '.') ? toc.substr(1) : toc;
		return (operator [](trimmed)).Navigate(restpath);
	} else if (type_ == array_type) {
		std::string toc = path;
		if (!toc.size() || toc.at(0) != '[')
			throw (std::runtime_error("Invalid path"));
		std::size_t toc_pos = toc.find_first_of("]", 1);
		if (toc_pos == std::string::npos)
			throw (std::runtime_error("Invalid path"));
		toc = toc.substr(1, toc_pos - 1);
		if (toc.empty())
			throw (std::runtime_error("Invalid path"));
		std::string restpath = path.substr(toc.size() + 2);
		size_t idx = atoi(toc.c_str());
		return (operator [](idx)).Navigate(restpath);
	}
	throw std::runtime_error("Invalid path");
}

void RpcProperty::SetProp(const rexjson::value& val)
{
	check_type(object_type);
	object_->SetProp(val);
}

rexjson::value RpcProperty::GetProp()
{
	check_type(object_type);
	return object_->GetProp();
}

void RpcProperty::EnumChildren(const std::string& path, const RpcPropertyCallback& callback)
{
	if (type_ == map_type) {
		for (auto &o : *map_) {
			o.second.EnumChildren(path + "." + o.first, callback);
		}
	} else if (type_ == array_type) {
		size_t i = 0;
		for (auto &o : *array_) {
			o.EnumChildren(path + "[" + std::to_string(i++) + "]", callback);
		}
	} else if (type_ == object_type) {
		callback(path, *this);
	}
}
