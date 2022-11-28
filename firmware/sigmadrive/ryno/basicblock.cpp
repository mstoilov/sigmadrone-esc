#include <iomanip>
#include <iostream>
#include "basicblock.h"

namespace ryno {

BasicBlock::BasicBlock(size_t id, const std::string& ruleid, const char* src_begin, const char* src_end,
	size_t level, size_t line, size_t column, size_t offset, cst::TagTreeNode::Type type) 
	: base(), cst::TagTreeNode(type), id_(id), ruleid_(ruleid), src_begin_(src_begin), src_end_(src_end), level_(level)
	, line_(line), column_(column), offset_(offset)
{
}

void BasicBlock::AddInstruction(asmins_t ins, bool reloc)
{
	if (reloc) {
		if (!(ins.opcode >= CPU_B && ins.opcode <= CPU_BL && ins.da))
			throw RelocationError("Invalid relocation request");
		ins.re = 1;
	}
	push_back(ins);
}

size_t BasicBlock::GetId() const
{
	return id_;
}

std::string BasicBlock::GetSrc() const
{
	if (src_begin_ == nullptr || src_end_ == nullptr)
		return "";
	return std::string(src_begin_, src_end_);
}

std::string BasicBlock::GetRuleId() const
{
	return ruleid_;
}

void BasicBlock::Dump(std::ostream& os) const
{
	os << ry_cpu_dump(data(), size());
}

std::string BasicBlock::repr(size_t maxstrsize) const
{
	auto repeat = [](const std::string& s, ssize_t n) -> std::string {
		std::string ret;
		for (ssize_t i = 0; i < n; i++)
			ret += s;
		return ret;
	};

	std::stringstream oss;
	std::string in;
	std::string src = GetSrc();
	if (src.length() > maxstrsize)
		in = src.substr(0, 40) + "...";
	else
		in = src;
	oss << "(" << std::setw(5) << line_ << " : " << std::setw(3) << column_ << ") :" << std::setw(6) << offset_;
	oss << std::setw(10) << src.length();
	oss << std::setw(20) << ruleid_;
	oss << " Level:" << std::setw(3) << level_ << " ";
	oss << "Id:" << std::setw(4) << std::hex << id_ << " ";
	oss << (type == Begin ? "BEGIN" : "END  ");
	oss << repeat("    ", level_) + "    "; 
	oss << in;
	std::string instr = ry_cpu_dump(data(), size());
	if (!instr.empty())
		oss << instr;
	return oss.str();
}


}
