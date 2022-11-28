#ifndef _CODEFRAGMENT_H_
#define _CODEFRAGMENT_H_

#include <list>
#include "arrayobject.h"
#include "basicblock.h"


namespace ryno {

class Compiler;

class CodeFragment {
public:
	using codecontainer = std::vector<BasicBlock>;
	using stringcontainer = std::list<std::string>;
	CodeFragment();
	CodeFragment(codecontainer::const_iterator begin, codecontainer::const_iterator end);
	CodeFragment(const CodeFragment&) = default;
	CodeFragment(const std::string& name,
				const std::vector<std::string>& arguments,
				const ArrayObject& objects,
				codecontainer::const_iterator begin, 
				codecontainer::const_iterator end);

	/**
	 * Insert BasicBlock into the basicblocks_ vector. Sets the current basic block to the
	 * newly created one.
	 *
	 * @param index 		Where to insert the block.
	 * @param blockid 		the id of the newly created block.
	 */
	void InsertBlockBefore(size_t index, size_t blockid);

	/**
	 * Add a new BasicBlock at the end. Sets the current basic block to the
	 * newly created one.
	 *
	 * @param blockid		Unique block Id.
	 * @param ruleid		Rule Id producing this block.
	 * @param src_begin		Source code, producing this block
	 * @param src_begin		Source code, producing this block
	 */
	void AddBlock(size_t blockid = 0, const std::string& ruleid = "", const char* src_begin = nullptr, const char* src_end = nullptr,
		size_t level = 0, size_t line = 0, size_t column = 0, size_t offset = 0, cst::TagTreeNode::Type type = cst::TagTreeNode::Begin);

	/**
	 * Add instruction to the current block. If the reloc is true
	 * the branch argument will be treated a blockid and it will be
	 * replaced by the correct offset of the specified block.
	 *
	 * @param ins			Instruction to be added to the current block
	 * @param reloc			Specify if this instruction argument needs to be treated
	 * as block id.
	 */
	void AddInstruction(asmins_t ins, bool reloc = false);

	/**
	 * Convert all basic blocks into a single block. All block reloc instructions
	 * ids will be converted to offsets.
	 */
	void Normalize();

	/**
	 * Return the index of the specified blockid. The index is zero based.
	 * If the block id cannot be found an exception will be thrown.
	 *
	 * @param blockid		The blockid of the basic block.
	 * @return				The index of the spcecified block.
	 */
	size_t FindBlockId(size_t blockid);

	/**
	 * Calculate the offset of the specified "blockid" from the instruction
	 * located in block specified by block index "nblock" and instruction index "nins".
	 *
	 * @param nblock		The index of the source basic block.
	 * @param nins			The index of the source instruction.
	 * @param blockid		blockid of the destination block.
	 * @return
	 */
	ssize_t BlockOffset(size_t nblock, size_t nins, size_t blockid);

	/**
	 * Reference to the current block.
	 *
	 * @return				Reference to the current block.
	 */
	BasicBlock& CurrentBlock();

	/**
	 * @brief Get the Unique Block Id object
	 * 
	 * @return size_t 
	 */
	size_t GetUniqueBlockId();

	/**
	 * @brief Dump the block
	 * 
	 */
	void Dump();

	/**
	 * @brief Add a new RyObject to the code fragment. Takes ownership of the object
	 * 
	 * @param obj 		Pointer to the new temp RyObject
	 * @return size_t 	The index of the newly added object
	 */
	size_t AddObject(RyObject* v);

	const std::string& GetName() const 
	{ 
		return name_; 
	}

	const std::vector<std::string>& GetArguments() const
	{
		return arguments_;
	}

	const ArrayObject& GetObjects() const
	{
		return objects_;
	}

public:
	ssize_t current_;
	codecontainer basicblocks_;
	size_t unique_block_id_;
	std::string name_;
	std::vector<std::string> arguments_;
	ArrayObject objects_;
};

}

#endif // _CODEFRAGMENT_H_
