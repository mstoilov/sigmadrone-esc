#include <sstream>
#include "compiler.h"
#include "functionobject.h"
#include "cst/tagtreenode.h"

namespace ryno
{

	template<typename Iterator>
	std::string escaped(Iterator begin, Iterator end)
	{
		std::string ret;
		for (Iterator it = begin; it < end; it++) {
			if (it < end && *it == '\\' && (it + 1) < end) {
				/*
				* Escaped char
				*/
				it++;
				std::string::value_type c = 0;
				switch(*it) {
				case 'r':
					c = '\r';
					break;
				case 'n':
					c = '\n';
					break;
				case 't':
					c = '\t';
					break;
				case '0':
					c = '\0';
					break;
				case 'f':
					c = '\f';
					break;
				case 'v':
					c = '\v';
					break;
				case '\"':
					c = '\"';
					break;
				case '\'':
					c = '\'';
					break;
				case '\\':
					c = '\\';
					break;
				default:
					c = *it;
					break;
				}
				ret.insert(ret.end(), c);
			} else {
				ret.insert(ret.end(), *it);
			}
		}
		return ret;
	}

	void Compiler::InitParserCode()
	{



	}

	Compiler::Compiler()
	{
		codegen_begin_["map"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(RYO_MAP, XX, XX, XX, 0));
		};
		codegen_begin_["array"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(RYO_ARRAY, XX, XX, XX, 0));
		};
		codegen_end_["mapelem"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R2, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_LDSREF, R0, SP, XX, 0));
			it->AddInstruction(ry_asm(RYO_ASSAT, R2, R0, R1, 0));
		};
		codegen_end_["arrayelem"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_LDSREF, R0, SP, XX, 0));
			it->AddInstruction(ry_asm(RYO_APPEND, R1, R0, XX, 0));
		};
		codegen_end_["decimal"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			RyObject *obj = new IntObject(std::strtol(std::string(it->src_begin_, it->src_end_).c_str(), nullptr, 10));
			r_uint_t idx = fragment.AddObject(obj);
			it->AddInstruction(ry_asm(RYO_AT, R0, CO, DA, idx));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["hex"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			RyObject *obj = new IntObject(std::strtol(std::string(it->src_begin_, it->src_end_).c_str(), nullptr, 16));
			r_uint_t idx = fragment.AddObject(obj);
			it->AddInstruction(ry_asm(RYO_AT, R0, CO, DA, idx));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["bin"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			RyObject *obj = new IntObject(std::strtol(std::string(it->src_begin_, it->src_end_).c_str(), nullptr, 2));
			r_uint_t idx = fragment.AddObject(obj);
			it->AddInstruction(ry_asm(RYO_AT, R0, CO, DA, idx));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["float"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			RyObject *obj = new FloatObject(std::strtod(std::string(it->src_begin_, it->src_end_).c_str(), nullptr));
			r_uint_t idx = fragment.AddObject(obj);
			it->AddInstruction(ry_asm(RYO_AT, R0, CO, DA, idx));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["boolconst"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			RyObject *obj = new BoolObject(std::string(it->src_begin_, it->src_end_));
			r_uint_t idx = fragment.AddObject(obj);
			it->AddInstruction(ry_asm(RYO_AT, R0, CO, DA, idx));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["stringconst"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			RyObject *obj = new StringObject(escaped(it->src_begin_ + 1, it->src_end_ - 1));
			r_uint_t idx = fragment.AddObject(obj);
			it->AddInstruction(ry_asm(RYO_AT, R0, CO, DA, idx));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["idname"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			RyObject *obj = new StringObject(std::string(it->src_begin_, it->src_end_));
			r_uint_t idx = fragment.AddObject(obj);
			it->AddInstruction(ry_asm(RYO_AT, R0, CO, DA, idx));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["assignexpr"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(RYO_ASSIGN, LP, DA, XX, cst::tagtree_nchildren(it, begin, end)));
		};
		codegen_end_["idref"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(RYO_REFEXT, LP, GP, DA, cst::tagtree_nchildren(it, begin, end)));
		};
		codegen_end_["toplevelexpr"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
		};
		codegen_end_["negterm"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_NEG, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["predec"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_DEC, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["preinc"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_INC, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["postdec"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_CLONE, R0, R1, XX, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_DEC, R1, XX, XX, 0));
		};
		codegen_end_["postinc"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_CLONE, R0, R1, XX, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_INC, R1, XX, XX, 0));
		};
		codegen_end_["nterm"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_NEG, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["leftshift"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_LSR, R0, R0, R1, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["add"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_ADD, R0, R1, R0, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["sub"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_SUB, R0, R0, R1, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["mul"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_MUL, R0, R0, R1, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["div"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_DIV, R0, R0, R1, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["eq"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_EQ, R0, R0, R1, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["ne"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_NE, R0, R0, R1, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["lt"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_LT, R0, R0, R1, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["gt"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_GT, R0, R0, R1, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["le"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_LE, R0, R0, R1, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};
		codegen_end_["ge"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_POP, R1, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_GE, R0, R0, R1, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};

		codegen_end_["if_cond"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			BIterator if_it = cst::tagtree_parent(it, begin, end, cst::TagTreeNode::Type::End);
			BIterator body_it = cst::tagtree_next(it, begin, end, cst::TagTreeNode::Type::Begin);
			BIterator next_it = cst::tagtree_next(body_it, begin, end, cst::TagTreeNode::Type::Begin);
			if (next_it == end)
				next_it = if_it;
			it->AddInstruction(ry_asm(RYO_BOOLEAN, XX, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_BNEQ, DA, XX, XX, next_it->GetId()), true);
		};
		codegen_begin_["if_body"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_CFLAG, DA, XX, XX, CPU_STATUS_Z));
		};
		codegen_end_["if_body"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			BIterator if_it = cst::tagtree_parent(it, begin, end, cst::TagTreeNode::Type::End);
			it->AddInstruction(ry_asm(CPU_B, DA, XX, XX, if_it->GetId()), true);
		};
		codegen_end_["while_cond"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			BIterator while_it = cst::tagtree_parent(it, begin, end, cst::TagTreeNode::Type::End);
			it->AddInstruction(ry_asm(RYO_BOOLEAN, XX, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_BNEQ, DA, XX, XX, while_it->GetId()), true);
		};
		codegen_end_["while_body"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			BIterator while_it = cst::tagtree_parent(it, begin, end, cst::TagTreeNode::Type::Begin);
			it->AddInstruction(ry_asm(CPU_B, DA, XX, XX, while_it->GetId()), true);
		};

		codegen_end_["for_cond"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			BIterator for_it = cst::tagtree_parent(it, begin, end, cst::TagTreeNode::Type::End);
			BIterator body_it = cst::tagtree_lastchild(for_it, begin, end, cst::TagTreeNode::Type::Begin);
			if (it->src_end_ != it->src_begin_) {
				it->AddInstruction(ry_asm(RYO_BOOLEAN, XX, XX, XX, 0));
				it->AddInstruction(ry_asm(CPU_POP, R0, XX, XX, 0));
				it->AddInstruction(ry_asm(CPU_BNEQ, DA, XX, XX, for_it->GetId()), true);
			}
			it->AddInstruction(ry_asm(CPU_B, DA, XX, XX, body_it->GetId()), true);
		};

		codegen_end_["for_postaction"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			BIterator cond_it = cst::tagtree_prev(it, begin, end, cst::TagTreeNode::Type::Begin);
			it->AddInstruction(ry_asm(CPU_B, DA, XX, XX, cond_it->GetId()), true);
		};

		codegen_end_["for_body"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			BIterator postaction_it = cst::tagtree_prev(it, begin, end, cst::TagTreeNode::Type::Begin);
			it->AddInstruction(ry_asm(CPU_B, DA, XX, XX, postaction_it->GetId()), true);
		};

		codegen_end_["break"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			BIterator loop_it = it;
			while ((loop_it = cst::tagtree_parent(loop_it, begin, end, cst::TagTreeNode::Type::End)) != end)
			{
				if (loop_it->ruleid_ == "for_statement" || loop_it->ruleid_ == "while_statement")
				{
					it->AddInstruction(ry_asm(CPU_B, DA, XX, XX, loop_it->GetId()), true);
					break;
				}
			}
			if (loop_it == end)
				throw SyntaxError(it->line_, it->column_, it->offset_, "Invalid 'break' statement");
		};

		codegen_end_["topassignexpr"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_DEC, SP, DA, XX, 0));
			it->AddInstruction(ry_asm(CPU_MOV, R0, DA, XX, 0));
		};

		codegen_end_["function_body"] =
		codegen_end_["functionblock"] =
		codegen_end_["if_statement"] =
		codegen_end_["while_statement"] =
		codegen_end_["for_statement"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			it->AddInstruction(ry_asm(CPU_MOV, R0, DA, XX, 0));
		};

		codegen_end_["continue"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			BIterator loop_it = it;
			while ((loop_it = cst::tagtree_parent(loop_it, begin, end, cst::TagTreeNode::Type::End)) != end)
			{
				if (loop_it->ruleid_ == "for_statement" || loop_it->ruleid_ == "while_statement")
				{
					BIterator body_it = cst::tagtree_lastchild(loop_it, begin, end, cst::TagTreeNode::Type::End);
					it->AddInstruction(ry_asm(CPU_B, DA, XX, XX, body_it->GetId()), true);
					break;
				}
			}
			if (loop_it == end)
				throw SyntaxError(it->line_, it->column_, it->offset_, "Invalid 'continue' statement");
		};

		codegen_end_["return"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			for (BIterator function_it = it; function_it != end; function_it++)
			{
				if ((function_it->ruleid_ == "anonfunction" || function_it->ruleid_ == "function") && function_it->type == cst::TagTreeNode::Type::End)
				{
					it->AddInstruction(ry_asm(CPU_B, DA, XX, XX, function_it->GetId()), true);
					return;
				}
			}
			throw SyntaxError(it->line_, it->column_, it->offset_, "Invalid 'return' statement");
		};

		codegen_begin_["anonfunction"] =
		codegen_begin_["function"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void {
			it->AddInstruction(ry_asm(RYO_MAP, XX, XX, XX, 0));
			it->AddInstruction(ry_asm(RYO_LDSREF, LP, SP, DA, 0));
		};

		codegen_end_["anonfunction"] = 
		codegen_end_["function"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void {
			/*
			 * Decrease SP to destroy the local variabls map
			 */
			it->AddInstruction(ry_asm(CPU_DEC, SP, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_MOV, R1, DA, XX, 0));
			it->AddInstruction(ry_asm(RYO_UNREF, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_RET, XX, XX, XX, 0));
		};

		codegen_begin_["funcname"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void {
			fragment.name_ = std::string(it->src_begin_, it->src_end_);
		};

		codegen_end_["call"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void {
			size_t nchildren = cst::tagtree_nchildren(it, begin, end);
			it->AddInstruction(ry_asm(CPU_SUB, FP, SP, DA, nchildren));
			it->AddInstruction(ry_asm(CPU_PUSHM, DA, XX, XX, BIT(CO)|BIT(LP)|BIT(FP)|BIT(LR)));
			it->AddInstruction(ry_asm(RYO_LDSREF, R0, FP, DA, 0 + 1));
			it->AddInstruction(ry_asm(RYO_CALL, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_POPM, DA, XX, XX, BIT(CO)|BIT(LP)|BIT(FP)|BIT(LR)));
			it->AddInstruction(ry_asm(CPU_MOV, SP, FP, XX, 0));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};

		codegen_begin_["funcarg"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void {
			fragment.arguments_.emplace_back(it->src_begin_, it->src_end_);
			size_t n = cst::tagtree_childindex(it, begin, end);
			RyObject *obj = new StringObject(std::string(it->src_begin_, it->src_end_));
			r_uint_t idx = fragment.AddObject(obj);
			it->AddInstruction(ry_asm(RYO_AT, R1, CO, DA, idx));
			it->AddInstruction(ry_asm(RYO_LDSREF, R0, FP, DA, n + 2));
			it->AddInstruction(ry_asm(RYO_ASSAT, R0, LP, R1, XX));
		};

		codegen_begin_["fragment"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void {
		};

		codegen_end_["fragment"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void {
			it->AddInstruction(ry_asm(RYO_UNREF, R0, XX, XX, 0));
			it->AddInstruction(ry_asm(CPU_MOV, R1, DA, XX, 0));
			it->AddInstruction(ry_asm(CPU_MOV, R2, DA, XX, 0));
			it->AddInstruction(ry_asm(CPU_EXT, XX, XX, XX, 0));
		};

		codegen_begin_["functionblock"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void {
			BIterator functionblock_end = cst::tagtree_get(it, begin, end, cst::TagTreeNode::Type::End);
			it->AddInstruction(ry_asm(CPU_B, DA, XX, XX, functionblock_end->GetId()), true);
		};

		codegen_end_["functionblock"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void
		{
			BIterator function = cst::tagtree_firstchild(it, begin, end, cst::TagTreeNode::Type::Begin);
			BIterator funcname = cst::tagtree_firstchild(function, begin, end, cst::TagTreeNode::Type::Begin);
			BIterator function_end = cst::tagtree_firstchild(it, begin, end, cst::TagTreeNode::Type::End);
			CodeFragment funcfrag(fragment.GetName(), fragment.GetArguments(), fragment.GetObjects(), function, function_end + 1);
			funcfrag.Normalize();
			size_t function_idx = fragment.AddObject(new FunctionObject(funcfrag));
			size_t funcname_idx = fragment.AddObject(new StringObject(std::string(funcname->src_begin_, funcname->src_end_)));
			it->AddInstruction(ry_asm(RYO_AT, R0, CO, DA, function_idx));
			it->AddInstruction(ry_asm(RYO_AT, R1, CO, DA, funcname_idx));
			it->AddInstruction(ry_asm(RYO_ASSAT, R0, LP, R1, funcname_idx));
			it->AddInstruction(ry_asm(CPU_MOV, R0, DA, XX, 0));
		};

		codegen_begin_["anonfunctionblock"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void {
			BIterator functionblock_end = cst::tagtree_get(it, begin, end, cst::TagTreeNode::Type::End);
			it->AddInstruction(ry_asm(CPU_B, DA, XX, XX, functionblock_end->GetId()), true);
		};

		codegen_end_["anonfunctionblock"] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void {
			BIterator function = cst::tagtree_firstchild(it, begin, end, cst::TagTreeNode::Type::Begin);
			BIterator function_end = cst::tagtree_firstchild(it, begin, end, cst::TagTreeNode::Type::End);
			CodeFragment funcfrag(fragment.GetName(), fragment.GetArguments(), fragment.GetObjects(), function, function_end + 1);
			funcfrag.Normalize();
			size_t idx = fragment.AddObject(new FunctionObject(funcfrag));
			it->AddInstruction(ry_asm(RYO_AT, R0, CO, DA, idx));
			it->AddInstruction(ry_asm(CPU_PUSH, R0, XX, XX, 0));
		};

		codegen_end_[""] = [](CodeFragment &fragment, BIterator it, BIterator begin, BIterator end) -> void {
		};
	}

	Compiler::~Compiler()
	{
	}

	void Compiler::ThrowErrorFromFailedRules(const std::vector<cst::line_column_offset_rule> &failed_rules)
	{
		if (!failed_rules.empty())
		{
			std::vector<cst::line_column_offset_rule>::const_reverse_iterator rit = failed_rules.rbegin();
			throw SyntaxError(rit->line(), rit->col(), rit->offset());
		}
		else
		{
			throw SyntaxError("Syntax Error at unknown location");
		}
	}

	AST Compiler::Parse(const std::string &in)
	{
		std::string::const_iterator it = in.begin();
		AST ast;

		while (it != in.end())
		{
			auto res = parser_.parse_extended("fragment", it, in.begin(), in.end());
			if (!res.status)
			{
				ThrowErrorFromFailedRules(res.failed_rules);
			}
			for (const auto &r : res.ast)
			{
				ast.push_back({r.level, r.begin, r.end, r.line, r.column, r.offset, r.ruleid, (size_t)-1});
			}
			it += res.length;
		}
		return ast;
	}

	size_t Compiler::ParseFragment(CodeFragment& code, std::string::const_iterator it, std::string::const_iterator begin, std::string::const_iterator end)
	{
		using RIterator = std::vector<cst::PRT_EntryEx<GrammarParser::Iterator>>::iterator;

		auto res = parser_.parse_extended("fragment", it, begin, end);
		if (!res.status)
			ThrowErrorFromFailedRules(res.failed_rules);
		
		cst::leveltree_walk(
			res.ast.begin(), res.ast.begin(), res.ast.end(),
			[&](RIterator it, RIterator begin, RIterator end)
			{
				code.AddBlock(code.GetUniqueBlockId(), it->ruleid, &*it->begin, &*it->end, it->level, it->line, it->column, it->offset, cst::TagTreeNode::Type::Begin);
			},
			[&](RIterator it, RIterator begin, RIterator end)
			{
				code.AddBlock(code.GetUniqueBlockId(), it->ruleid, &*it->begin, &*it->end, it->level, it->line, it->column, it->offset, cst::TagTreeNode::Type::End);
			});
		return res.length;
	}

	void Compiler::CompileFragment(CodeFragment &code)
	{
		BIterator b = code.basicblocks_.begin();
		BIterator e = code.basicblocks_.end();

		for (BIterator it = b; it != e; it++)
		{
			if (it->type == cst::TagTreeNode::Type::Begin)
			{
				auto id_handler = codegen_begin_.find(it->ruleid_);
				if (id_handler != codegen_begin_.end())
				{
					id_handler->second(code, it, b, e);
				}
			}
			else if (it->type == cst::TagTreeNode::Type::End)
			{
				auto id_handler = codegen_end_.find(it->ruleid_);
				if (id_handler != codegen_end_.end())
				{
					id_handler->second(code, it, b, e);
				}
			}
		}
	}

}
