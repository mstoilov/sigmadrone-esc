#ifndef _CSTENTRY_H_
#define _CSTENTRY_H_

#include <string>
#include <vector>
#include <stddef.h>
#include <iomanip>
#include <locale>
#include <codecvt>

#include "leveltreenode.h"

namespace cst
{
	
	using RuleIdMap = std::map<size_t, std::string>;

	template<typename Iterator>
	struct  PRT_Entry : public LevelTreeNode {
		using LevelTreeNode::level;
		using LevelTreeNode::GetLevel;
		using LevelTreeNode::SetLevel;

		PRT_Entry() : LevelTreeNode(-1), serial(0) {}
		PRT_Entry(size_t level, size_t serialval, Iterator b, Iterator e) 
			: LevelTreeNode(level), serial(serialval), begin(b), end(e) {}

		size_t serial;				// The serial id of the rule emmiting the entry
		Iterator begin;				// Start of the matched string
		Iterator end;				// End of the matched string

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
			oss << "Serial: " << std::setw(7) << serial;
			oss << std::setw(10) << length();
			oss << " Level: " << std::setw(3) << level;
			oss << repeat("    ", level) + "    "; 
			oss << in;
			return oss.str();
		}

	};

	template<typename Iterator>
	struct PRT_EntryEx : public LevelTreeNode {
		using LevelTreeNode::level;
		using LevelTreeNode::GetLevel;
		using LevelTreeNode::SetLevel;


		size_t serial;				// The serial id of the rule emmiting the entry
		Iterator begin;				// Start of the matched string
		Iterator end;				// End of the matched string
		size_t line;				// Line number
		size_t column;				// Column number
		size_t offset;				// Offset of the matched string
		std::string ruleid;			// Rule name
		void* userdata;				// Scratch area

		PRT_EntryEx(size_t levelval, size_t serialval, Iterator b, Iterator e, size_t lineval, size_t columnval,
			size_t offsetval, const std::string& ruleidval, void* userdataval)
			: LevelTreeNode(levelval), serial(serialval), begin(b), end(e)
			, line(lineval), column(columnval), offset(offsetval), ruleid(ruleidval), userdata(userdataval) {}
		PRT_EntryEx() : LevelTreeNode(), serial(0UL), line(0UL), column(0UL), offset(0UL), userdata(nullptr) {}

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

} // namespace cst

#endif
