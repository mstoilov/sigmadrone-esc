#include <iostream>
#include <iomanip>
#include "codefragment.h"
#include "compiler.h"

namespace ryno {

CodeFragment::CodeFragment()
	: current_(-1), unique_block_id_(0)
{
}

CodeFragment::CodeFragment(codecontainer::const_iterator begin, codecontainer::const_iterator end) : CodeFragment()
{
	std::copy(begin, end, std::back_inserter(basicblocks_));
}

CodeFragment::CodeFragment(const std::string& name,
				const std::vector<std::string>& arguments,
				const ArrayObject& objects,
				codecontainer::const_iterator begin, 
				codecontainer::const_iterator end)
				: current_(-1)
				, unique_block_id_(0)
				, name_(name)
				, arguments_(arguments)
				, objects_(objects)
{
	std::copy(begin, end, std::back_inserter(basicblocks_));
	if (basicblocks_.size())
		current_ = basicblocks_.size() - 1;
}

void CodeFragment::InsertBlockBefore(size_t index, size_t blockid)
{
	codecontainer::iterator it = basicblocks_.begin() + index;
	basicblocks_.insert(it, BasicBlock(blockid));
	current_ = index;
}

void CodeFragment::AddBlock(size_t blockid, const std::string& ruleid, const char* src_begin, const char* src_end,
	size_t level, size_t line, size_t column, size_t offset, cst::TagTreeNode::Type type)
{
	current_ = basicblocks_.size();
	basicblocks_.emplace_back(blockid, ruleid, src_begin, src_end, level, line, column, offset, type);
}

void CodeFragment::AddInstruction(asmins_t ins, bool reloc)
{
	if (reloc) {
		if (!(ins.opcode >= CPU_B && ins.opcode <= CPU_BL && ins.da))
			throw RelocationError("Invalid relocation request");
		ins.re = 1;
	}
	if (basicblocks_.empty())
		AddBlock();
	CurrentBlock().push_back(ins);
}

BasicBlock& CodeFragment::CurrentBlock()
{
	if (basicblocks_.empty())
		throw std::runtime_error("CodeFragment has no basic blocks.");
	return basicblocks_[current_];
}

void CodeFragment::Normalize()
{
	BasicBlock tmpblock;

	for (size_t nblock = 0; nblock < basicblocks_.size(); nblock++) {
		for (size_t nins = 0; nins < basicblocks_[nblock].size(); nins++) {
			asmins_t ins = basicblocks_[nblock][nins];

			/*
			 * Handle branching to another basic block.
			 */
			if (ins.re) {
				r_int_t offset = (r_int_t)BlockOffset(nblock, nins, ins.data);
				ins.data = (r_uint_t)offset;
				ins.re = 0;
			}

			tmpblock.push_back(ins);
		}
	}
	basicblocks_.clear();
	basicblocks_.emplace_back(tmpblock);
	current_ = 0;
}

size_t CodeFragment::FindBlockId(size_t blockid)
{
	for (size_t nblock = 0; nblock < basicblocks_.size(); nblock++) {
		if (basicblocks_[nblock].GetId() == blockid)
			return nblock;
	}
	throw OutOfRangeError(std::string("Failed to find blockid: " + std::to_string(blockid)));
}

ssize_t CodeFragment::BlockOffset(size_t nblock, size_t nins, size_t blockid)
{
	ssize_t offset = 0;
	size_t ntarget = FindBlockId(blockid);
	if (ntarget >= basicblocks_.size())
		throw(OutOfRangeError("Target block index is larger than the number of blocks"));
	if (ntarget > nblock) {
		offset = basicblocks_[nblock].size() - nins;
		for (size_t i = nblock + 1; i < ntarget; i++)
			offset += basicblocks_[i].size();
	} else {
		offset = nins;
		for (size_t i = ntarget; i < nblock; i++)
			offset += basicblocks_[i].size();
		offset = -offset;
	}
	return offset;
}

size_t CodeFragment::GetUniqueBlockId()
{
	return unique_block_id_++;
}

size_t CodeFragment::AddObject(RyObject* v)
{
	return objects_.emplace_back(v);
}

void CodeFragment::Dump()
{
	size_t index = 0;

	for (auto& block : basicblocks_) {
		std::cout << std::setw(4) << index++ << ".";
		std::cout << " Block id: 0x" << std::hex << block.GetId();
		std::cout << " [ " << block.GetRuleId() << " ] ";
		std::cout << block.GetSrc() << std::endl;
		block.Dump(std::cout);
	}
}


}
