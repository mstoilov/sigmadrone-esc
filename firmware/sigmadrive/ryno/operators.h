#ifndef _OPPLUS_H_
#define _OPPLUS_H_

#include "ryvariant.h"

namespace ryno {

struct AddVisitor {
	template<typename A, typename B>
	RyVariant operator()(const A& a, const B& b)
	{
		if constexpr ((std::is_same<A, RyInt>::value || std::is_same<A, RyFloat>::value) &&
				(std::is_same<B, RyInt>::value || std::is_same<B, RyFloat>::value)) {
			return RyVariant(a + b);
		} else if constexpr (std::is_same<A, RyStringPtr>::value && std::is_same<B, RyStringPtr>::value) {
			return RyVariant(*a + *b);
		} else if constexpr (std::is_same<A, RyArrayPtr>::value && std::is_same<B, RyArrayPtr>::value) {
			if (a->size() != b->size())
				throw("Invalid add operation, the dimensions of the arrays must be the same.");
			RyArrayPtr ret;
			for (size_t i = 0; i < a->size(); i++)
				ret->push_back(a->at(i) + b->at(i));
			return ret;
		} else if constexpr ((std::is_same<A, RyArrayPtr>::value) ) {
			RyArrayPtr ret;
			for (size_t i = 0; i < a->size(); i++)
				ret->push_back(a->at(i) + b);
			return ret;
		} else {
			throw std::runtime_error("Incompatible types supplied for add operation.");
		}
	}
};


struct SubtractVisitor {
	template<typename A, typename B>
	RyVariant operator()(const A& a, const B& b)
	{
		if constexpr ((std::is_same<A, RyInt>::value || std::is_same<A, RyFloat>::value) &&
				(std::is_same<B, RyInt>::value || std::is_same<B, RyFloat>::value)) {
			return RyVariant(a - b);
		} else if constexpr (std::is_same<A, RyArrayPtr>::value && std::is_same<B, RyArrayPtr>::value) {
			if (a->size() != b->size())
				throw("Invalid subract operation, the dimensions of the arrays must be the same.");
			RyArrayPtr ret;
			for (size_t i = 0; i < a->size(); i++)
				ret->push_back(a->at(i) - b->at(i));
			return ret;
		} else if constexpr ((std::is_same<A, RyArrayPtr>::value) ) {
			RyArrayPtr ret;
			for (size_t i = 0; i < a->size(); i++)
				ret->push_back(a->at(i) - b);
			return ret;
		} else {
			throw std::runtime_error("Incompatible types supplied for subtract operation.");
		}
	}
};


struct MultiplyVisitor {
	template<typename A, typename B>
	RyVariant operator()(const A& a, const B& b)
	{
		if constexpr ((std::is_same<A, RyInt>::value || std::is_same<A, RyFloat>::value) &&
				(std::is_same<B, RyInt>::value || std::is_same<B, RyFloat>::value)) {
			return RyVariant(a * b);
		} else if constexpr ((std::is_same<A, RyInt>::value && std::is_same<B, RyStringPtr>::value) ) {
			smart_ptr<typename B::element_type> ret;
			for (ssize_t i = 0; a > 0 && i < a; i++)
				ret->insert(ret->end(), b->begin(), b->end());
			return ret;
		} else if constexpr (std::is_same<A, RyStringPtr>::value && std::is_same<B, RyInt>::value) {
			smart_ptr<typename A::element_type> ret;
			for (ssize_t i = 0; b > 0 && i < b; i++)
				ret->insert(ret->end(), a->begin(), a->end());
			return ret;
		} else if constexpr ((std::is_same<B, RyArrayPtr>::value) ) {
			RyArrayPtr ret;
			for (size_t i = 0; i < b->size(); i++)
				ret->push_back(a * b->at(i));
			return ret;
		} else if constexpr (std::is_same<A, RyArrayPtr>::value) {
			RyArrayPtr ret;
			for (size_t i = 0; i < a->size(); i++)
				ret->push_back(a->at(i) * b);
			return ret;
		} else {
			throw std::runtime_error("Incompatible types supplied for multiply operation.");
		}



	}
};


struct DivideVisitor {
	template<typename A, typename B>
	RyVariant operator()(const A& a, const B& b)
	{
		if constexpr ((std::is_same<A, RyInt>::value || std::is_same<A, RyFloat>::value) &&
				(std::is_same<B, RyInt>::value || std::is_same<B, RyFloat>::value)) {
			if (!b)
				throw std::runtime_error("Division by zero.");
			return RyVariant(a / b);
		} else if constexpr (std::is_same<A, RyArrayPtr>::value) {
			RyArrayPtr ret;
			for (size_t i = 0; i < a->size(); i++)
				ret->push_back(a->at(i) / b);
			return ret;
		} else {
			throw std::runtime_error("Incompatible types supplied for divide operation.");
		}
	}
};

}

#endif
