#ifndef _COMPILER_H_
#define _COMPILER_H_


#include <map>
#include <functional>
#include "grammarparser.h"
#include "module.h"


namespace ryno {


	template<typename Iterator>
	struct ASTNodeType {
		size_t level;				// Tree level
		Iterator begin;				// Start of the matched string
		Iterator end;				// End of the matched string
		size_t line;				// Line number
		size_t column;				// Column number
		size_t offset;				// Offset of the matched string
		std::string ruleid;			// Rule name
		size_t blockid;				// Block Id of the basic block mapped to this node

		typename std::iterator_traits<Iterator>::difference_type length() const
		{
			return end - begin;
		}

		std::string repr(size_t maxstrsize = 40) const
		{
			auto repeat = [](const std::string& s, ssize_t n) -> std::string {
				std::string ret;
				for (ssize_t i = 0; i < n; i++)
					ret += s;
				return ret;
			};

			std::stringstream oss;
			std::string in;
			if (static_cast<size_t>(length()) > maxstrsize) {
				if constexpr (std::is_same<wchar_t, std::remove_cv_t<typename std::iterator_traits<Iterator>::value_type>>::value) {
					std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
					in = converter.to_bytes(std::wstring(begin, begin + maxstrsize));
				} else {
					in = std::string(begin, begin + maxstrsize);
				}
				in +=  + "...";
			} else {
				if constexpr (std::is_same<wchar_t, std::remove_cv_t<typename std::iterator_traits<Iterator>::value_type>>::value) {
					std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
					in = converter.to_bytes(std::wstring(begin, end));
				} else {
					in = std::string(begin, end);
				}
			}
			oss << "(" << std::setw(5) << line << " : " << std::setw(3) << column << ") :" << std::setw(6) << offset;
			oss << std::setw(10) << length();
			oss << std::setw(20) << ruleid;
			oss << " Level:" << std::setw(3) << level;
			oss << repeat("    ", level) + "    "; 
			oss << in;
			return oss.str();
		}
	};


using ASTNode = ASTNodeType<std::string::const_iterator>;
using AST = std::vector<ASTNode>;

class Compiler
{
private:
	/* data */
public:
	Compiler();
	~Compiler();

	AST Parse(const std::string& in);
	size_t ParseFragment(CodeFragment& code, std::string::const_iterator it, std::string::const_iterator begin, std::string::const_iterator end);
	void CompileFragment(CodeFragment &code);
	void dump_rules(std::ostream& os)
	{
	    parser_.dump_rules(os);
	}

protected:
	void InitParserCode();
	static void ThrowErrorFromFailedRules(const std::vector<cst::line_column_offset_rule>& failed_rules);

	using BIterator = CodeFragment::codecontainer::iterator;

	std::map<std::string, std::function<void(CodeFragment& fragment, BIterator it, BIterator begin, BIterator end)>> codegen_begin_;
	std::map<std::string, std::function<void(CodeFragment& fragment, BIterator it, BIterator begin, BIterator end)>> codegen_end_;

	GrammarParser parser_;
};

}

#endif
