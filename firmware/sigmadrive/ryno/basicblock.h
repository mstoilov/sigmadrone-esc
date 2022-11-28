#ifndef _BASICBLOCK_H_
#define _BASICBLOCK_H_

#include <vector>

#include "cst/tagtreenode.h"
#include "rycpu.h"

namespace ryno {


class BasicBlock : public std::vector<asmins_t>, cst::TagTreeNode {
public:
	using base = std::vector<asmins_t>;
	using base::vector;
	using base::emplace_back;
	using base::push_back;
	using cst::TagTreeNode::type;

	BasicBlock(size_t id = 0, const std::string& ruleid = "", const char* src_begin = nullptr, const char* src_end = nullptr, 
		size_t level = 0, size_t line = 0, size_t column = 0, size_t offset = 0, Type type = Begin);
	void AddInstruction(asmins_t ins, bool reloc = false);	
	size_t GetId() const;
	std::string GetRuleId() const;
	std::string GetSrc() const;
	void Dump(std::ostream& os) const;
	std::string repr(size_t maxstrsize = 40) const;

public:
	size_t id_;
	std::string ruleid_;
	const char* src_begin_;
	const char* src_end_;
	size_t level_;				// Tree level
	size_t line_;				// Line number
	size_t column_;				// Column number
	size_t offset_;				// Offset of the matched string
};


}

#endif	// _BASICBLOCK_H_
