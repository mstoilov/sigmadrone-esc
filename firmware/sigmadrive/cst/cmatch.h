#ifndef _CMATCH_H_
#define _CMATCH_H_

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <tuple>
#include <memory>
#include <map>
#include <regex>


namespace cst
{
	template <typename Iterator>
	int mbtowc(wchar_t *pwc, Iterator input, Iterator end)
	{
		int n;
		unsigned char c;

		if (input >= end)
		{
			*pwc = (wchar_t)0;
			return 0;
		}

		if ((c = (unsigned char)input[0]) < 0x80)
		{
			*pwc = c;
			return 1;
		}
		n = (int)(end - input);
		if (c == 0xC0 || c == 0xC1 || (c >= 0xF5))
			goto error;
		if ((c >> 5) == 6)
		{
			if (n < 2 || ((unsigned char)input[1] >> 6 != 0x02))
				goto error;
			*pwc = ((unsigned char)(c & 0x1f) << 6) | (unsigned char)(input[1] ^ 0x80);
			return 2;
		}
		else if ((c >> 4) == 0x0E)
		{
			if (n < 3 || ((unsigned char)input[1] >> 6 != 0x02) || ((unsigned char)input[2] >> 6 != 0x02))
				goto error;
			*pwc = ((wchar_t)(c & 0x0f) << 12) | ((wchar_t)((unsigned char)input[1] ^ 0x80) << 6) | (wchar_t)((unsigned char)input[2] ^ 0x80);
			return 3;
		}
		else if ((c >> 3) == 0x1E)
		{
			if (n < 4 || ((unsigned char)input[1] >> 6 != 0x02) || (unsigned char)(input[2] >> 6 != 0x02) || ((unsigned char)input[3] >> 6 != 0x02))
				goto error;
			*pwc = ((wchar_t)(c & 0x07) << 18) | ((wchar_t)((unsigned char)input[1] ^ 0x80) << 12) | ((wchar_t)((unsigned char)input[2] ^ 0x80) << 6) | (wchar_t)((unsigned char)input[3] ^ 0x80);
			return 4;
		}

	error:
		*pwc = c;
		return 1;
	}


	struct line_column
	{
		size_t nline;
		size_t ncol;

		line_column(size_t line, size_t col) : nline{line}, ncol{col} {}
		line_column() : nline{0}, ncol{0} {}

		size_t line() const
		{
			return nline;
		}

		void line(size_t val)
		{
			nline = val;
		}

		size_t col() const
		{
			return ncol;
		}

		void col(size_t val)
		{
			ncol = val;
		}
	};

	struct line_column_offset : public line_column
	{
		size_t noffset;

		line_column_offset() : line_column(), noffset{0} {}
		line_column_offset(size_t line, size_t col, size_t offset) : line_column(line, col), noffset{offset} {}

		size_t offset() const
		{
			return noffset;
		}

		void offset(size_t val)
		{
			noffset = val;
		}
	};

	struct line_column_offset_rule : public line_column_offset
	{
		std::string ruleid;

		line_column_offset_rule(size_t line, size_t col, size_t offset, const std::string &rule)
			: line_column_offset(line, col, offset), ruleid{rule} {}
		line_column_offset_rule(const line_column_offset& lco, const std::string &rule)
			: line_column_offset(lco), ruleid{rule} {}

		std::string rule() const
		{
			return ruleid;
		}

		void rule(std::string val)
		{
			ruleid = val;
		}
	};

	struct offset_serial
	{
		size_t noffset;
		size_t nserial;

		offset_serial() : noffset{0}, nserial{0} {}
		offset_serial(size_t offset, size_t serial) : noffset{offset}, nserial{serial} {}

		size_t offset() const
		{
			return noffset;
		}

		void offset(size_t val)
		{
			noffset = val;
		}

		size_t serial() const
		{
			return nserial;
		}

		void serial(size_t val)
		{
			nserial = val;
		}

		bool operator==(const offset_serial &key) const
		{
			return (key.noffset == noffset && key.nserial == nserial) ? true : false;
		}

		bool operator!=(const offset_serial &key) const
		{
			return !operator==(key);
		}
	};

	struct parse_exception : public line_column_offset, std::runtime_error
	{
		parse_exception(const std::string& msg, size_t line, size_t column, size_t offset)
			: line_column_offset(line, column, offset), std::runtime_error(msg) {}
		parse_exception(const std::string& msg, const line_column_offset& lco) 
			: line_column_offset(lco), std::runtime_error(msg) {}
	};

	struct parse_error
	{
		std::string msg;
		size_t line;
		size_t column;
		size_t offset;
	};

	/**
	 * @brief Return the offsets of the new lines.
	 *
	 * @tparam Iterator
	 * @param begin Start of the input stream
	 * @param end End of the input stream
	 * @return std::vector<size_t>
	 */
	template <typename Iterator>
	std::vector<size_t> newline_scan(Iterator begin, Iterator end)
	{
		std::vector<size_t> nlvec;

		nlvec.push_back(0);
		for (Iterator it = begin; it < end; it++)
		{
			if (*it == '\n')
			{
				nlvec.push_back(it - begin + 1);
			}
		}
		return nlvec;
	}

	template <typename Iterator>
	line_column get_line_column(
		const std::vector<size_t> &newline_offsets,
		size_t offset,
		Iterator begin,
		Iterator end,
		size_t tab = 4)
	{
		size_t line = 0;
		size_t col = 0;
		std::vector<size_t>::const_reverse_iterator nlo_it = newline_offsets.rbegin();
		for (; nlo_it < newline_offsets.rend(); nlo_it++)
		{
			if (offset >= *nlo_it)
			{
				line = newline_offsets.rend() - nlo_it;
				break;
			}
		}
		/*
		 * Find any tab characters and increase the column based on
		 * the position of the tab: 4 - (col % 4)
		 */
		Iterator it = begin + *nlo_it;
		while (it < begin + offset && it < end)
		{
			if constexpr (std::is_same<wchar_t, std::remove_cv_t<typename std::iterator_traits<Iterator>::value_type>>::value) {
				col += (*it == '\t') ? tab - (col % tab) : 1;
				it += 1;
			} else {
				wchar_t c;
				int charwidth = mbtowc(&c, it, end);
				col += (*it == '\t') ? tab - (col % tab) : 1;
				it += charwidth;
			} 
		}
		/*
		 * Finally convert from 0-starting to 1-starting offset.
		 */
		col += 1;
		return line_column{line, col};
	}

	template <typename Iterator>
	line_column_offset get_line_column_offset(
		const std::vector<size_t> &newline_offsets,
		size_t offset,
		Iterator begin,
		Iterator end,
		size_t tab = 4)
	{
		line_column lc = get_line_column(newline_offsets, offset, begin, end, tab);
		return line_column_offset(lc.line(), lc.col(), offset);
	}

	template<typename Iterator>
	struct inputstream
	{
		Iterator it;
		Iterator begin;
		Iterator end;
		size_t tabsize;

		inputstream(const Iterator& i, const Iterator& b, const Iterator& e, size_t tabval = 4) 
			: it{i}, begin{b}, end{e}, tabsize{tabval} {}
	
		void fwd(size_t val)
		{
			it += val;
		}

		size_t pos() const noexcept
		{
			return it - begin;
		}

	};

	/**
	 * @brief The base class for all other matching classes.
	 *
	 * The matching objects are either brancher or leafs.
	 * Branch objects recursively invoke other matching objects.
	 * 		For example: CAnd, COr, CCat, ...
	 *
	 * Leaf objects try to match whatever their function is
	 * to the input stream current iterator position.
	 * 		For example: CMBChar, CMBRange, CRegEx
	 *
	 * When the matching objects hold pointers to their children
	 * they don't have to manage their lifetime. This is done
	 * by the parser.
	 *
	 * The base class for all other matching classes. It also holds
	 * the common @ref serial.
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CMatch
	{
		CMatch(Parser *pp) : parser(pp), serial(pp->get_serial()) {}
		virtual ~CMatch() {}
		/**
		 * @brief All matching objects must override this operator
		 * and implement their matching logic there. In case of
		 * successful match the @ref fit iterator would be increased
		 * and the 'true' is returned. In the case the matching
		 * fails the @ref fit iterator is left unchanged and
		 * 'false' is returned.
		 *
		 * @param fit 			The input stream
		 * @return true 		The match is successful
		 * @return false 		The match failed
		 */
		virtual bool operator()(typename Parser::InputStream &fit) const 
		{
			return false;
		}

		/**
		 * @brief This function is used as part of the construction of
		 * the matching tree representation. The matching object
		 * doesn't know how far from the root it is. This information
		 * is provided by the @level parameter and it is used
		 * to properly indent the output representation.
		 *
		 * @param os 				Output stream
		 * @param level 			The tree level (indentation level)
		 * @return std::ostream& 	Reference to the output stream
		 */
		virtual std::ostream &dump(std::ostream &os, size_t level) const
		{
			dump_level(os, level) << "CMatch" << std::endl;
			return os;
		}

		/**
		 * @brief Helper function used to construct the matching tree
		 * representation. It produceses the indentation for the
		 * specified 'level' of the node in the matching tree.
		 *
		 * @param os 				Output stream
		 * @param level 			The tree level (indentation level)
		 * @return std::ostream& 	Reference to the output stream
		 */
		inline std::ostream &dump_level(std::ostream &os, size_t level) const
		{
			for (size_t i = 0; i < level; i++)
				os << "    ";
			return os;
		}

		Parser *parser; /**< Pointer to the @ref Parser class*/
		size_t serial;	/**< Unique identification of the match object in the parser match tree */
	};

	/**
	 * @brief This matching object will throw an exception
	 * if its child fails to match.
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CThrow : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		using CMatch<Parser>::serial;

		CThrow(Parser *pp, const typename Parser::MatchType *match, const std::string &msg)
			: CMatch<Parser>(pp), match_(match), msg_(msg) {}

		virtual bool operator()(typename Parser::InputStream &is) const override
		{
			typename Parser::InputStream cis(is);
			if (!(*match_)(is))
			{
				parser->failed_at(cis.pos(), serial);
				std::vector<size_t> nloff = newline_scan(is.begin, is.end);
				cst::line_column_offset lco(get_line_column_offset(nloff, cis.pos(), cis.begin, cis.end, cis.tabsize));
				cst::parse_exception err(msg_, lco);
				throw(err);
			}
			return true;
		}

		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CThrow: \"" << msg_ << "\"" << std::endl;
			match_->dump(os, level + 1);
			return os;
		}

		const typename Parser::MatchType *match_;
		std::string msg_;
	};


	/**
	 * @brief Match a single multi-byte character
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CMBChar : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		CMBChar(Parser *pp, wchar_t c) : CMatch<Parser>(pp), c_(c) {}

		virtual bool operator()(typename Parser::InputStream &is) const override
		{
			bool res = false;
			wchar_t wc = 0;
			int len = parser->get_char(&wc, is.it, is.end);
			if (len > 0 && wc == c_)
			{
				is.fwd(len);
				res = true;
			}
			return res;
		}

		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CMBChar: ";
			if (!std::isprint(c_))
				os << "#" << (unsigned long)c_ << std::endl;
			else
				os << '\'' << (char)c_ << '\'' << std::endl;
			return os;
		}
		wchar_t c_;
	};

	/**
	 * @brief Match any character except '\n'
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CMBAnyChar : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		CMBAnyChar(Parser *pp) : CMatch<Parser>(pp) {}
		virtual bool operator()(typename Parser::InputStream &is) const override
		{
			bool res = false;
			wchar_t wc = 0;
			int len = parser->get_char(&wc, is.it, is.end);
			if (len > 0 && wc != '\n')
			{
				is.fwd(len);
				res = true;
			}
			return res;
		}

		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CMBAnyChar " << std::endl;
			return os;
		}
	};

	/**
	 * @brief Match a range of characters.
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CMBRange : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		CMBRange(Parser *pp, wchar_t c1, wchar_t c2) : CMatch<Parser>(pp), c1_(c1), c2_(c2) {}

		virtual bool operator()(typename Parser::InputStream &is) const override
		{
			wchar_t wc = 0;
			int len = parser->get_char(&wc, is.it, is.end);
			if (len > 0 && wc >= c1_ && wc <= c2_)
			{
				is.fwd(len);
				return true;
			}
			return false;
		}

		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CMBRange: ";
			if (!std::isprint(c1_) || !std::isprint(c2_)) {
				os << "[#" << (unsigned long)c1_ << "-#" << (unsigned long)c2_ << "]" << std::endl;
			} else {
				os << "[" << (char) c1_ << "-" << (char) c2_ << "]" << std::endl;
			}
			return os;
		}

		wchar_t c1_;
		wchar_t c2_;
	};

	/**
	 * @brief Match a string
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CStr : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		CStr(Parser *pp, const std::string &s) : CMatch<Parser>(pp)
		{
			for (std::string::const_iterator str_it = s.begin(); str_it != s.end(); ) {
				wchar_t c;
				int charwidth = mbtowc(&c, str_it, s.end());
				str_.push_back(c);
				str_it += charwidth;
			}
		}

		virtual bool operator()(typename Parser::InputStream &is) const override
		{
			for (const auto& c : str_) {
				wchar_t wc = 0;
				int len = parser->get_char(&wc, is.it, is.end);
				if (len <= 0 || wc != c)
					return false;
				is.fwd(len);
			}
			return true;
		}

		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CString: ";
			for (const auto& c : str_)
				os << (char) c;
			os << std::endl;
			return os;
		}

		std::wstring str_;
	};

	/**
	 * @brief Match the start of a line. This matching object is
	 * used for the implementation of the line start (^) operator
	 * For Example: ^<term>
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CLineStart : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		CLineStart(Parser *pp, const typename Parser::MatchType *match) : CMatch<Parser>(pp), match_(match) {}
		virtual bool operator()(typename Parser::InputStream &is) const override
		{
			if (is.it == is.begin || *(is.it - 1) == '\n')
				return (*match_)(is);
			return false;
		}
		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CLineStart: " << std::endl;
			return match_->dump(os, level + 1);
		}
		const typename Parser::MatchType *match_;
	};

	/**
	 * @brief Match the end of a line. This matching object is
	 * used for the implementation of the line end ($) operator
	 * For Example: <term>$
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CLineEnd : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		CLineEnd(Parser *pp, const typename Parser::MatchType *match) : CMatch<Parser>(pp), match_(match) {}
		virtual bool operator()(typename Parser::InputStream &is) const override
		{
			typename Parser::InputStream cis(is);
			bool res = (*match_)(is);
			if (!res)
				return false;
			if (is.it != is.end && *is.it != '\n' && *is.it != '\r') {
				is = cis;
				return false;
			}
			return true;
		}
		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CLineEnd: " << std::endl;
			return match_->dump(os, level + 1);
		}
		const typename Parser::MatchType *match_;
	};


	/**
	 * @brief MatchMulClass implements the "one or more" ('+') operator like:
	 * <term>+
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CMul : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		CMul(Parser *pp, const typename Parser::MatchType *match) : CMatch<Parser>(pp), match_(match) {}
		virtual bool operator()(typename Parser::InputStream &is) const override
		{
			if (!(*match_)(is))
			{
				return false;
			}
			typename Parser::InputStream cis(is);
			size_t cst_top = parser->cst_size();
			while ((*match_)(is) && is.it != cis.it)
			{
				cst_top = parser->cst_size();
				cis = is;
			}

			/*
			 * Reset the tree to the last successful match
			 */
			parser->cst_resize(cst_top);
			is = cis;
			return true;
		}

		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CMul: " << std::endl;
			return match_->dump(os, level + 1);
		}
		const typename Parser::MatchType *match_;
	};

	/**
	 * @brief COpt implements the "one or zero" ('?') operator like:
	 * <term>?
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct COpt : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		COpt(Parser *pp, const typename Parser::MatchType *match) : CMatch<Parser>(pp), match_(match) {}
		virtual bool operator()(typename Parser::InputStream &fit) const override
		{
			size_t cst_top = parser->cst_size();
			typename Parser::InputStream tmp = fit;

			if (!(*match_)(fit))
			{
				fit = tmp;
				/*
				 * Reset the tree to the previous state.
				 */
				parser->cst_resize(cst_top);
			}
			return true;
		}

		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "COpt: " << std::endl;
			return match_->dump(os, level + 1);
		}

		const typename Parser::MatchType *match_;
	};

	/**
	 * @brief CMulOpt implements the "one or zero" ('?') operator like:
	 * <term>?
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CMulOpt : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		CMulOpt(Parser *pp, const typename Parser::MatchType *match) : CMatch<Parser>(pp), match_(match) {}
		virtual bool operator()(typename Parser::InputStream &fit) const override
		{
			size_t cst_top = parser->cst_size();
			typename Parser::InputStream cit = fit;

			if (!(*match_)(fit))
			{
				fit = cit;
				parser->cst_resize(cst_top);
				return true;
			}
			cit = fit;
			cst_top = parser->cst_size();
			while ((*match_)(fit) && cit.it != fit.it)
			{
				cst_top = parser->cst_size();
				cit = fit;
			}

			/*
			 * Reset the tree to the last successful match
			 */
			parser->cst_resize(cst_top);
			fit = cit;
			return true;
		}

		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CMulOpt: " << std::endl;
			return match_->dump(os, level + 1);
		}

		const typename Parser::MatchType *match_;
	};

	/**
	 * @brief Operator NOT.
	 * For example:
	 * !<term>
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CNot : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		CNot(Parser *pp, const typename Parser::MatchType *match) : CMatch<Parser>(pp), match_(match) {}
		virtual bool operator()(typename Parser::InputStream &fit) const override
		{
			size_t cst_top = parser->cst_size();
			typename Parser::InputStream cit = fit;
			if ((*match_)(cit) || cit.it == cit.end)
			{
				return false;
			}
			wchar_t wc = 0;
			int len = parser->get_char(&wc, fit.it, fit.end);
			if (len > 0)
				fit.fwd(len);
			else
				fit.fwd(1);
			parser->cst_resize(cst_top);
			return true;
		}
		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CNot: " << std::endl;
			return match_->dump(os, level + 1);
		}
		const typename Parser::MatchType *match_;
	};

	/**
	 * @brief Match sequence of terms like:
	 * <term1> <term2> <term3>
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CCat : CMatch<Parser>
	{
		using CMatch<Parser>::parser;

		CCat(Parser *pp) : CMatch<Parser>(pp) {}

		template <typename... ARGS>
		CCat(Parser *pp, ARGS &&...children) : CMatch<Parser>(pp)
		{
			insert_children(children...);
		}

		virtual bool operator()(typename Parser::InputStream &fit) const override
		{
			bool res = true;

			if (children_.empty())
			{
				throw(std::runtime_error("CCat is empty."));
			}

			for (auto &c : children_)
			{
				if (!(*c)(fit))
				{
					res = false;
					break;
				}
			}
			return res;
		}
		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CCat: " << std::endl;
			for (const auto &c : children_)
				c->dump(os, level + 1);
			return os;
		}

		template <typename ARG>
		void insert_children(ARG &&child)
		{
			children_.push_back(child);
		}

		template <typename ARG, typename... ARGS>
		void insert_children(ARG &&child, ARGS &&...children)
		{
			insert_children(child);
			insert_children(children...);
		}

		std::vector<typename Parser::MatchType *> children_;
	};

	/**
	 * @brief Match sequence of terms like:
	 * <term1> <term2> <term3>
	 * after every successful match of one of the terms
	 * it will try to match the 'automatch' object.
	 * This is used to implement matching of sequences with
	 * optional white spaces between the terms.
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CAutoCat : CMatch<Parser>
	{
		using CMatch<Parser>::parser;

		CAutoCat(Parser *pp, const typename Parser::MatchType *automatch) : CMatch<Parser>(pp), automatch_(automatch) {}

		template <typename... ARGS>
		CAutoCat(Parser *pp, const typename Parser::MatchType *automatch, ARGS &&...children) : CMatch<Parser>(pp), automatch_(automatch)
		{
			insert_children(children...);
		}

		virtual bool operator()(typename Parser::InputStream &fit) const override
		{
			bool res = true;
			if (children_.empty())
			{
				throw(std::runtime_error("CAutoCat is empty."));
			}
			for (auto &c : children_)
			{
				if (!(*c)(fit))
				{
					res = false;
					break;
				}
				if (automatch_)
					(*automatch_)(fit);
			}
			return res;
		}

		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CAutoCat: " << std::endl;
			for (const auto &c : children_)
				c->dump(os, level + 1);
			return os;
		}

		template <typename ARG>
		void insert_children(ARG &&child)
		{
			children_.push_back(child);
		}

		template <typename ARG, typename... ARGS>
		void insert_children(ARG &&child, ARGS &&...children)
		{
			insert_children(child);
			insert_children(children...);
		}

		std::vector<typename Parser::MatchType *> children_;
		const typename Parser::MatchType *automatch_;
	};

	/**
	 * @brief Match one of the terms like:
	 * <term1> | <term3>
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct COr : CMatch<Parser>
	{
		using CMatch<Parser>::parser;

		COr(Parser *pp) : CMatch<Parser>(pp) {}

		template <typename... ARGS>
		COr(Parser *pp, ARGS &&...children) : CMatch<Parser>(pp)
		{
			insert_children(children...);
		}

		virtual bool operator()(typename Parser::InputStream &fit) const override
		{
			bool res = false;
			typename Parser::InputStream cit(fit);
			size_t cst_top = parser->cst_size();
			for (auto &c : children_)
			{
				if ((*c)(fit))
				{
					res = true;
					break;
				}
				fit = cit;
				parser->cst_resize(cst_top);
			}
			return res;
		}

		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "COr: " << std::endl;
			for (const auto &c : children_)
				c->dump(os, level + 1);
			return os;
		}

		template <typename ARG>
		void insert_children(ARG &&child)
		{
			children_.push_back(child);
		}

		template <typename ARG, typename... ARGS>
		void insert_children(ARG &&child, ARGS &&...children)
		{
			insert_children(child);
			insert_children(children...);
		}

		std::vector<typename Parser::MatchType *> children_;
	};

	/**
	 * @brief Match all of the terms like:
	 * <term1> && <term3> && <term3>
	 *
	 * How much the iterator will be moved forward
	 * will be decided by the last term
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CAnd : CMatch<Parser>
	{
		using CMatch<Parser>::parser;

		CAnd(Parser *pp) : CMatch<Parser>(pp) {}

		template <typename... ARGS>
		CAnd(Parser *pp, ARGS &&...children) : CMatch<Parser>(pp)
		{
			insert_children(children...);
		}

		virtual bool operator()(typename Parser::InputStream &fit) const override
		{
			typename Parser::InputStream cit(fit);
			size_t cst_top = parser->cst_size();
			bool res = true;

			for (auto &c : children_)
			{
				parser->cst_resize(cst_top);
				fit = cit;
				if (!(*c)(fit))
				{
					res = false;
					break;
				}
			}
			return res;
		}

		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CAnd: " << std::endl;
			for (const auto &c : children_)
				c->dump(os, level + 1);
			return os;
		}

		template <typename ARG>
		void insert_children(ARG &&child)
		{
			children_.push_back(child);
		}

		template <typename ARG, typename... ARGS>
		void insert_children(ARG &&child, ARGS &&...children)
		{
			insert_children(child);
			insert_children(children...);
		}

		std::vector<typename Parser::MatchType *> children_;
	};

	/**
	 * @brief Call a named rule
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CCall : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		CCall(Parser *pp, const std::string &ruleid) : CMatch<Parser>(pp), ruleid_(ruleid), callrule_(nullptr)
		{
			// typename Parser::RuleMap::const_iterator it = parser->rules_.find(ruleid);
			// if (it != parser->rules_.end())
			// {
			// 	callrule_ = it->second;
			// }
		}
		virtual bool operator()(typename Parser::InputStream &fit) const override
		{
			bool res = false;
			res = parser->apply_rule(callrule_, fit);
			return res;
		}
		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CCall: " << ruleid_ << std::endl;
			return os;
		}
		std::string ruleid_;
		const CMatch<Parser> *callrule_;
	};

	/**
	 * @brief Emit a CST entry in the syntax tree upon sucessful match
	 *
	 * @tparam Parser
	 */
	template <typename Parser>
	struct CEmit : CMatch<Parser>
	{
		using CMatch<Parser>::parser;
		CEmit(Parser *pp, const std::string &ruleid, const typename Parser::MatchType *match) : CMatch<Parser>(pp), ruleid_(ruleid), match_(match) {}
		virtual bool operator()(typename Parser::InputStream &fit) const override
		{
			typename Parser::InputStream cit(fit);
			parser->cstlevel_inc();
			bool res = (*match_)(fit);
			parser->cstlevel_dec();
			if (res)
			{
				parser->emit(parser->cstlevel_get(), this->serial, cit.it, fit.it);
			}
			return res;
		}
		virtual std::ostream &dump(std::ostream &os, size_t level) const override
		{
			this->dump_level(os, level) << "CEmit: " << ruleid_ << std::endl;
			return match_->dump(os, level + 1);
		}
		std::string ruleid_;
		const typename Parser::MatchType *match_;
	};

}

#endif // _CMATCH_H_
