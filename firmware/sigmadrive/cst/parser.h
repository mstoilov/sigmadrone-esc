#ifndef _CSTPARSER_H_
#define _CSTPARSER_H_

#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <sstream>
#include <tuple>
#include <memory>
#include <map>
#include <exception>

#include "prtentry.h"
#include "cmemo.h"
#include "cmatch.h"

namespace cst
{
	// struct compile_error : public parse_exception
	// {
	// 	compile_error(const std::string &msg, size_t offset, size_t line, size_t column, const std::vector<line_column_offset_rule>& v) : parse_exception(msg, offset, line, column), ids(v) {}
	// 	std::vector<line_column_offset_rule> ids;
	// };

	template <typename InputIterator>
	class parser
	{
	public:
		using Iterator = InputIterator;
		using InputStream = cst::inputstream<Iterator>;
		using MatchType = CMatch<parser>;
		using RuleMap = std::map<std::string, const MatchType *>;
		using LRKey = cst::offset_serial;
		using InvolvedVector = std::vector<size_t>;
		using CST_VectorType = std::vector<PRT_Entry<Iterator>>;

		struct parse_result
		{
			bool status;
			size_t length;
			std::vector<PRT_Entry<Iterator>> ast;
			std::vector<line_column_offset_rule> failed_rules;
			parse_error error;
		};

		struct parse_result_ex
		{
			bool status;
			size_t length;
			std::vector<PRT_EntryEx<Iterator>> ast;
			std::vector<line_column_offset_rule> failed_rules;
			parse_error error;
		};

		struct HEAD
		{
			LRKey key;				 /*< The key is defined by {the offset, serial} pair */
			Iterator seed;			 /*< Progress of the LR rule */
			InvolvedVector involved; /*< All the indirect LR rules involved */
		};

		parser() : cstlevel_(0), serial_(0), autocat_(nullptr) {}
		virtual ~parser() {}
		void emit(size_t cstlevel, size_t serial, const Iterator &start, const Iterator &end);
		void compile(const std::string &bnf);
		void compile(const char* bnf);
		template <typename BNFIT>
		void compile(BNFIT begin, BNFIT end);
		void link();
		bool apply_root(const std::string &ruleid, InputStream &fit);
		bool apply_rule(const std::string &ruleid, InputStream &fit);
		bool apply_rule(const MatchType *m, InputStream &fit);

		parse_result parse(const std::string &ruleid, Iterator it, Iterator begin, Iterator end, bool withcst = true, size_t tabsize = 4);
		parse_result_ex parse_extended(const std::string &ruleid, Iterator it, Iterator begin, Iterator end, bool withcst = true, size_t tabsize = 4);
		static std::ostream &failed_dump(std::ostream &os, const std::vector<line_column_offset_rule> &failed);
		void dump_rules(std::ostream &os);
		void dump_rule(std::ostream &os, const std::string &rulename);
		void dump_compiler_rules(std::ostream &os);

		auto Char(uint32_t c)
		{
			rulesvec_.emplace_back(new CMBChar<parser>(this, c));
			return rulesvec_.rbegin()->get();
		}
		auto Any()
		{
			rulesvec_.emplace_back(new CMBAnyChar<parser>(this));
			return rulesvec_.rbegin()->get();
		}
		auto Range(uint32_t c1, uint32_t c2)
		{
			rulesvec_.emplace_back(new CMBRange<parser>(this, c1, c2));
			return rulesvec_.rbegin()->get();
		}
		auto Str(const std::string &s)
		{
			rulesvec_.emplace_back(new CStr<parser>(this, s));
			return rulesvec_.rbegin()->get();
		}
		auto LineStart(const MatchType *match)
		{
			rulesvec_.emplace_back(new CLineStart<parser>(this, match));
			return rulesvec_.rbegin()->get();
		}
		auto LineEnd(const MatchType *match)
		{
			rulesvec_.emplace_back(new CLineEnd<parser>(this, match));
			return rulesvec_.rbegin()->get();
		}
		auto Throw(const MatchType *match, const std::string &msg)
		{
			rulesvec_.emplace_back(new CThrow<parser>(this, match, msg));
			return rulesvec_.rbegin()->get();
		}
		auto Mul(const MatchType *match)
		{
			rulesvec_.emplace_back(new CMul<parser>(this, match));
			return rulesvec_.rbegin()->get();
		}
		auto Opt(const MatchType *match)
		{
			rulesvec_.emplace_back(new COpt<parser>(this, match));
			return rulesvec_.rbegin()->get();
		}
		auto MulOpt(const MatchType *match)
		{
			rulesvec_.emplace_back(new CMulOpt<parser>(this, match));
			return rulesvec_.rbegin()->get();
		}
		auto Emit(const std::string &ruleid, const MatchType *match)
		{
			rulesvec_.emplace_back(new CEmit<parser>(this, ruleid, match));
			return rulesvec_.rbegin()->get();
		}

		/**
		 * @brief Create a call to a named rule
		 *
		 * @param ruleid 	The name of the rule
		 * @return auto 	Pointer to the newly created Call object
		 */
		auto Call(const std::string &ruleid)
		{
			rulesvec_.emplace_back(new CCall<parser>(this, ruleid));
			callvec_.push_back((CCall<parser> *)rulesvec_.rbegin()->get());
			return rulesvec_.rbegin()->get();
		}

		/**
		 * @brief Creates an emitting named rule
		 * This object would emit a CST entry upon successful match
		 * Used for the implementation of ':=' operator
		 *
		 * @param ruleid 		Name of the rule
		 * @param match 		The underlying match object
		 * @return auto
		 */
		auto Rule(const std::string &ruleid, const MatchType *match)
		{
			if (rules_.find(ruleid) != rules_.end())
			{
				std::stringstream oss;
				oss << "ERROR: "
					<< "Rule named "
					<< "'" << ruleid << "'"
					<< " is already defined.";
				throw(std::runtime_error(oss.str()));
			}
			rulesvec_.emplace_back(new CEmit<parser>(this, ruleid, match));
			rules_[ruleid] = rulesvec_.rbegin()->get();
			ruleids_[rulesvec_.rbegin()->get()->serial] = ruleid;
			return rulesvec_.rbegin()->get();
		}

		/**
		 * @brief Creates an non emitting named rule
		 * This object would NOT emit a CST entry upon successful match
		 * Used for the implementation of '::=' operator
		 *
		 * @param ruleid 		Name of the rule
		 * @param match 		The underlying match object
		 * @return auto
		 */
		auto RuleN(const std::string &ruleid, const MatchType *match)
		{
			if (rules_.find(ruleid) != rules_.end())
			{
				std::stringstream oss;
				oss << "ERROR: "
					<< "Rule named "
					<< "'" << ruleid << "'"
					<< " is already defined.";
				throw(std::runtime_error(oss.str()));
			}
			rules_[ruleid] = match;
			ruleids_[match->serial] = ruleid;
			return match;
		}
		void AutoSkipOn(const MatchType *match) { autocat_ = match; }
		void AutoSkipOff() { autocat_ = nullptr; }

		template <typename... ARGS>
		auto Cat(ARGS &&...args)
		{
			if (autocat_)
				rulesvec_.emplace_back(new CAutoCat<parser>(this, autocat_, std::forward<ARGS>(args)...));
			else
				rulesvec_.emplace_back(new CCat<parser>(this, std::forward<ARGS>(args)...));
			return rulesvec_.rbegin()->get();
		}

		template <typename... ARGS>
		auto Or(ARGS &&...args)
		{
			rulesvec_.emplace_back(new COr<parser>(this, std::forward<ARGS>(args)...));
			return rulesvec_.rbegin()->get();
		}
		template <typename... ARGS>
		auto And(ARGS &&...args)
		{
			rulesvec_.emplace_back(new CAnd<parser>(this, std::forward<ARGS>(args)...));
			return rulesvec_.rbegin()->get();
		}
		auto Not(const MatchType *match)
		{
			rulesvec_.emplace_back(new CNot<parser>(this, match));
			return rulesvec_.rbegin()->get();
		}

	protected:
		template <typename Parser>
		friend struct CMatch;
		template <typename Parser>
		friend struct CThrow;
		template <typename Parser>
		friend struct CMBChar;
		template <typename Parser>
		friend struct CMBAnyChar;
		template <typename Parser>
		friend struct CMBRange;
		template <typename Parser>
		friend struct CStr;
		template <typename Parser>
		friend struct CLineStart;
		template <typename Parser>
		friend struct CLineEnd;
		template <typename Parser>
		friend struct CMul;
		template <typename Parser>
		friend struct COpt;
		template <typename Parser>
		friend struct CMulOpt;
		template <typename Parser>
		friend struct CNot;
		template <typename Parser>
		friend struct CCat;
		template <typename Parser>
		friend struct CAutoCat;
		template <typename Parser>
		friend struct COr;
		template <typename Parser>
		friend struct CAnd;
		template <typename Parser>
		friend struct CCall;
		template <typename Parser>
		friend struct CEmit;

		template <typename CIterator>
		void compile_entry(CIterator it, CIterator begin, CIterator end);

		size_t get_serial()
		{
			return ++serial_;
		}

		size_t cst_size() const
		{
			return prt_.size();
		}

		void cst_resize(size_t size)
		{
			prt_.resize(size);
		}

		void cstlevel_inc()
		{
			cstlevel_++;
		}

		void cstlevel_dec()
		{
			cstlevel_--;
		}

		size_t cstlevel_get()
		{
			return cstlevel_;
		}

		void clear_rule_id_map() { prt_.ruleids_.clear(); }

		void head_push(const HEAD &h) { head_.push_back(h); }
		void head_pop() { head_.resize(head_.size() - 1); }
		bool head_empty() { return head_.empty(); }
		auto head_top() { return head_.rbegin(); }

		void invokation_push(const LRKey &k) { stack_invocation_.push_back(k); }
		void invokation_pop() { stack_invocation_.resize(stack_invocation_.size() - 1); }
		bool invokation_empty() { return stack_invocation_.empty(); }
		auto invokation_top() { return stack_invocation_.rbegin(); }
		void memo_add(size_t pos, size_t rule, bool res, size_t length, size_t csttop);
		const MemoEntry<Iterator> *memo_lookup(size_t pos, size_t rule) const;

		std::string get_rule_id(size_t serial) const;
		InvolvedVector get_involved_lr(const LRKey &key);
		bool check_involved_lr(size_t offset, size_t serial);
		bool grow_lr(typename std::vector<HEAD>::reverse_iterator head_ri, const MatchType *m, InputStream &fit, size_t asttop);
		bool check_lr(const LRKey &key);

		std::vector<line_column_offset_rule> get_failed(Iterator begin, Iterator end, size_t tabsize);
		void failed_at(size_t pos, size_t serial);
		int get_char(wchar_t *pwc, Iterator input, Iterator end);

	protected:
		size_t cstlevel_;
		size_t serial_;
		RuleIdMap ruleids_;
		std::vector<PRT_Entry<Iterator>> prt_;
		std::vector<std::unique_ptr<MatchType>> rulesvec_;
		std::vector<CCall<parser> *> callvec_;
		RuleMap rules_;
		std::vector<LRKey> stack_invocation_;
		std::vector<LRKey> stack_failedat_;
		std::vector<HEAD> head_;
		CMemo<Iterator> memo_;
		const MatchType *autocat_;
	};

	template <typename Iterator>
	std::vector<line_column_offset_rule> parser<Iterator>::get_failed(Iterator begin, Iterator end, size_t tabsize)
	{
		std::vector<line_column_offset_rule> failed_rules;
		std::vector<size_t> newline_offsets = newline_scan(begin, end);
		for (auto &i : stack_failedat_)
		{
			line_column_offset lco = get_line_column_offset(newline_offsets, i.offset(), begin, end, tabsize);
			failed_rules.push_back(line_column_offset_rule(lco, get_rule_id(i.serial())));
		}
		return failed_rules;
	}

	template <typename Iterator>
	std::string parser<Iterator>::get_rule_id(size_t serial) const
	{
		typename RuleIdMap::const_iterator it = ruleids_.find(serial);
		if (it == ruleids_.end())
		{
			std::stringstream oss;
			oss << "ERROR: "
				<< "RuleId for serial: "
				<< "'" << serial << "'"
				<< ", is not defined.";
			throw(std::runtime_error(oss.str()));
		}
		return it->second;
	}

	template <typename InputIterator>
	void parser<InputIterator>::memo_add(size_t pos, size_t rule, bool res, size_t length, size_t csttop)
	{
		memo_.add(pos, rule, res, length, prt_.begin() + csttop, prt_.end(), cstlevel_);
	}

	template <typename InputIterator>
	const MemoEntry<InputIterator> *parser<InputIterator>::memo_lookup(size_t pos, size_t rule) const
	{
		return memo_.lookup(pos, rule);
	}

	/**
	 * @brief Resolve rule names in the Call objects.
	 *
	 * During compilation of the parse tree 'Call' objects
	 * might refer to rules that are not defined yet. This function
	 * would fix this problem.
	 *
	 * @tparam Iterator
	 */
	template <typename Iterator>
	void parser<Iterator>::link()
	{
		for (auto &c : callvec_)
		{
			typename RuleMap::const_iterator it = rules_.find(c->ruleid_);
			if (it == rules_.end())
			{
				std::stringstream oss;
				oss << "ERROR: "
					<< "Rule named "
					<< "'" << c->ruleid_ << "'"
					<< " is not defined.";
				throw(std::runtime_error(oss.str()));
			}
			c->callrule_ = it->second;
		}
		callvec_.clear();
	}

	/**
	 * @brief Same as apply_rule, but this function is
	 * required to increase the iterator. If the iterator is 
	 * not increased a successful match of the underlying rule 
	 * will still be treated as failure.
	 *
	 * @tparam Iterator
	 * @param ruleid                Name of the rule
	 * @param fit                   Input stream iterator
	 * @return true                 Success
	 * @return false                Failure
	 */
	template <typename Iterator>
	bool parser<Iterator>::apply_root(const std::string &ruleid, InputStream &fit)
	{
		size_t pos = fit.pos();
		bool res = apply_rule(ruleid, fit);
		if (fit.pos() == pos)
			res = false;
		return res;
	}

	/**
	 * @brief Match ruleid to the input stream at the current
	 * position of fit iterator.
	 *
	 * @tparam Iterator
	 * @param ruleid 		Name of the rule
	 * @param fit 			Input stream iterator
	 * @return true 		Success
	 * @return false 		Failure
	 */
	template <typename Iterator>
	bool parser<Iterator>::apply_rule(const std::string &ruleid, InputStream &fit)
	{
		typename RuleMap::const_iterator it = rules_.find(ruleid);
		if (it == rules_.end())
		{
			std::stringstream oss;
			oss << "ERROR: "
				<< "Rule named "
				<< "'" << ruleid << "'"
				<< " is not defined.";
			throw(std::runtime_error(oss.str()));
		}

		const MatchType *m = it->second;
		return apply_rule(m, fit);
	}

	/**
	 * @brief Match a matching object to the input stream at the current
	 * position of fit iterator.
	 *
	 * @tparam Iterator
	 * @param m 			Pointer to the matching object
	 * @param fit 			Input stream iterator
	 * @return true 		Success
	 * @return false 		Failure
	 */
	template <typename Iterator>
	bool parser<Iterator>::apply_rule(const MatchType *m, InputStream &fit)
	{
		InputStream cit(fit);
		size_t pos = cit.pos();
		size_t ast_top = prt_.size();
		LRKey key{pos, m->serial};
		bool res = false;

		if (!head_empty() && head_top()->key == key)
		{
			fit.it = head_top()->seed;
			return (fit.it == cit.it) ? false : true;
		}

		if (!check_lr(key))
		{
#ifndef NO_MEMOIZATION
			const MemoEntry<Iterator> *memo = memo_lookup(pos, m->serial);
			if (memo)
			{
				if (memo->result)
				{
					for (auto c : memo->prt)
					{
						c.level += cstlevel_;
						prt_.push_back(c);
					}
					fit.fwd(memo->length);
				}
				return memo->result;
			}
#endif
			invokation_push(key);
			if ((res = (*m)(fit)) == false)
				failed_at(pos, m->serial);
#ifndef NO_MEMOIZATION
			if (head_empty() || head_top()->key.offset() != pos)
			{
				memo_add(pos, m->serial, res, res ? (size_t)(fit.it - cit.it) : 0, ast_top);
			}
#endif
			invokation_pop();

			/*
			 * Check if the rule is the head of the LR
			 */
			if (!head_empty() && head_top()->key == key)
			{
				/*
				 * Save the current matched iterator in the seed
				 */
				head_top()->seed = fit.it;
				// prt_.resize(ast_top);

				/*
				 * Grow the LR head rule. Reset the iterator
				 * in the begining (fit = cit).
				 */
				fit = cit;
				res = grow_lr(head_top(), m, fit, ast_top);
				head_pop();
				goto end;
			}
		}
		else
		{
			head_push({key, fit.it, get_involved_lr(key)});
		}

	end:
		return res;
	}

	/**
	 * @brief Check for left recursion
	 *
	 * @tparam Iterator
	 * @param key
	 * @return true
	 * @return false
	 */
	template <typename Iterator>
	bool parser<Iterator>::check_lr(const LRKey &key)
	{
		for (typename std::vector<LRKey>::reverse_iterator ri = stack_invocation_.rbegin();
			 ri != stack_invocation_.rend() && ri->offset() == key.offset(); ri++)
		{
			if (key == *ri)
				return true;
		}
		return false;
	}

	/**
	 * @brief Grow left recursion rule
	 *
	 * @tparam Iterator
	 * @param head_ri
	 * @param m
	 * @param fit
	 * @return true
	 * @return false
	 */
	template <typename Iterator>
	bool parser<Iterator>::grow_lr(typename std::vector<HEAD>::reverse_iterator head_ri, const MatchType *m, InputStream &fit, size_t asttop)
	{
		InputStream cit(fit);
		size_t pos = cit.pos();

		while (true)
		{
			size_t top = prt_.size();
			fit = cit;
			bool res = (*m)(fit);
			if (!res)
				return false;
			if (fit.it <= head_ri->seed)
			{
				/*
				 * Didn't match at the tail, reset the iterator
				 * with the value currently stored in the seed.
				 */
				fit.it = head_ri->seed;
				prt_.resize(top);
				break;
			}
			/*
			 * Save the current matched iterator in the seed
			 */
			head_ri->seed = fit.it;

			/*
			 * Synthesize the tree
			 */
			if (top < prt_.size() && top > asttop)
			{
				ssize_t delta = 0;
				std::string ruleid = get_rule_id(prt_[top].serial);
				if (check_involved_lr(pos, prt_[top].serial))
					delta = prt_[top].level - prt_[top - 1].level;
				for (size_t i = asttop; i < top; i++)
				{
					prt_[i].level += 1 + delta;
				}
			}
		}
		return true;
	}

	/**
	 * @brief Return the unique ids of the rules involved in left recursion.
	 *
	 * @tparam Iterator
	 * @param key
	 * @return parser<Iterator>::InvolvedVector
	 */
	template <typename Iterator>
	typename parser<Iterator>::InvolvedVector parser<Iterator>::get_involved_lr(const LRKey &key)
	{
		InvolvedVector involved;

		for (typename std::vector<LRKey>::reverse_iterator ri = stack_invocation_.rbegin();
			 ri != stack_invocation_.rend() && *ri != key; ri++)
		{
			involved.push_back(ri->serial());
		}
		return involved;
	}

	/**
	 * @brief Check if a rule is involved in left recursion.
	 *
	 * @tparam Iterator
	 * @param offset
	 * @param serial
	 * @return true
	 * @return false
	 */
	template <typename Iterator>
	bool parser<Iterator>::check_involved_lr(size_t offset, size_t serial)
	{
		if (head_empty())
			return false;
		for (const auto &r : head_top()->involved)
		{
			if (r == serial)
				return true;
		}
		return false;
	}

	template <typename Iterator>
	void parser<Iterator>::emit(size_t cstlevel, size_t serial, const Iterator &start, const Iterator &end)
	{
		prt_.push_back(PRT_Entry<Iterator>(cstlevel, serial, start, end));
	}

	template <typename Iterator>
	void parser<Iterator>::failed_at(size_t pos, size_t serial)
	{
		typename std::vector<LRKey>::const_reverse_iterator it = stack_failedat_.rbegin();
		if (stack_failedat_.empty() || pos > it->offset() || (pos == it->offset() && stack_invocation_.size() > stack_failedat_.size()))
		{
			stack_failedat_.clear();
			stack_failedat_.insert(stack_failedat_.cbegin(), stack_invocation_.begin(), stack_invocation_.end());
		}
	}

	template <typename Iterator, typename CB>
	Iterator parse_records_to_tree(Iterator rec, Iterator begin, Iterator end, const CB &cb)
	{
		cb(rec, begin, end);
		for (Iterator it = lastchild(rec, begin, end); it != end; it = prev(it, begin, end))
		{
			parse_records_to_tree(it, begin, end, cb);
		}
		return next(rec, begin, end);
	}

	template <typename T>
	void add_begin_tag(T &v)
	{
		size_t size = v.size();
		for (size_t i = 0; i < size * 2; i += 2)
		{
			typename T::iterator cur = v.begin() + i;
			typename T::iterator prev = std::prev(cur);
			size_t cur_level = cur->level;
			while (prev != std::prev(v.begin()) && cur_level < prev->level)
			{
				cur = prev;
				prev = std::prev(cur);
			}
			typename T::value_type tmp = *(v.begin() + i);
			// tmp.type = EntryType::Begin;
			v.insert(cur, tmp);
		}
	}

	template <typename T>
	void make_tree_from_records(T &v)
	{
		size_t size = v.size();
		if (!size)
			return;
		for (size_t i = size - 1; i >= 1;)
		{
			bool swapped = false;
			for (ssize_t j = i; j >= 1; j--)
			{
				if (v[j].level < v[j - 1].level)
				{
					std::swap(v[j], v[j - 1]);
					swapped = true;
				}
				else
				{
					break;
				}
			}
			if (!swapped)
				i--;
		}
	}

	template <typename Iterator>
	typename parser<Iterator>::parse_result parser<Iterator>::parse(const std::string &ruleid, Iterator it, Iterator begin, Iterator end, bool withcst, size_t tabsize)
	{
		parse_result ret{false, 0UL};
		InputStream fit(it, begin, end);
		cstlevel_ = 0;
		prt_.clear();
		stack_invocation_.clear();
		stack_failedat_.clear();
		memo_.invalidate();
		try
		{
			ret.status = apply_root(ruleid, fit);
			ret.length = fit.it - it;
			if (withcst && ret.status && !prt_.empty())
			{
				make_tree_from_records(prt_);
				using CstIterator = typename std::vector<PRT_Entry<Iterator>>::iterator;
				for (CstIterator it = prt_.begin(); it != prt_.end(); it++)
					ret.ast.push_back(*it);
			}
			if (!ret.status)
				ret.failed_rules = get_failed(begin, end, tabsize);
		}
		catch (parse_exception &e)
		{
			ret.length = 0;
			ret.status = false;
			ret.failed_rules = get_failed(begin, end, tabsize);
			ret.error = parse_error{e.what(), e.line(), e.col(), e.offset()};
		}
		return ret;
	}

	template <typename Iterator>
	typename parser<Iterator>::parse_result_ex parser<Iterator>::parse_extended(const std::string &ruleid, Iterator it, Iterator begin, Iterator end, bool withcst, size_t tabsize)
	{
		parse_result ret = parse(ruleid, it, begin, end, withcst, tabsize);
		parse_result_ex retex;

		retex.status = ret.status;
		retex.length = ret.length;
		retex.error = ret.error;
		retex.failed_rules = ret.failed_rules;
		if (!ret.ast.empty()) {
			std::vector<size_t> newline_offsets = newline_scan(begin, end);
			for (auto &r : ret.ast)
			{
				size_t offset = r.begin - begin;
				line_column lc = get_line_column(newline_offsets, offset, begin, end, tabsize);
				retex.ast.push_back({r.level, r.serial, r.begin, r.end, lc.line(), lc.col(), offset, get_rule_id(r.serial), nullptr});
			}
		}
		return retex;
	}

	template <typename Iterator>
	void parser<Iterator>::dump_rules(std::ostream &os)
	{
		for (const auto &r : rules_)
		{
			dump_rule(os, r.first);
			os << std::endl;
		}
	}

	template <typename Iterator>
	void parser<Iterator>::dump_rule(std::ostream &os, const std::string &rulename)
	{
		auto r = rules_.find(rulename);
		if (r != rules_.end())
		{
			os << r->first << " ->" << std::endl;
			r->second->dump(os, 1);
		}
	}

	template <typename Iterator>
	int parser<Iterator>::get_char(wchar_t *pwc, Iterator input, Iterator end)
	{
		if constexpr (std::is_same<char, std::remove_cv_t<typename std::iterator_traits<Iterator>::value_type>>::value)
		{
			return mbtowc(pwc, input, end);
		}
		else if constexpr (std::is_same<wchar_t, std::remove_cv_t<typename std::iterator_traits<Iterator>::value_type>>::value)
		{
			if (input < end)
			{
				*pwc = *input;
				return 1;
			}
		}
		return 0;
	}

}

#include "ebnfcompiler.h"
namespace cst
{
	template <typename Iterator>
	void parser<Iterator>::dump_compiler_rules(std::ostream &os)
	{
		cst::EBNF_Compiler<std::string::const_iterator> ebnf_parser;
		ebnf_parser.dump_rules(os);
	}

	template <typename Iterator>
	template <typename CIterator>
	void parser<Iterator>::compile_entry(CIterator it, CIterator begin, CIterator end)
	{
		if (it->ruleid == "char")
		{
			std::string tmp = escaped(it->begin, it->end);
			wchar_t c = 0;
			mbtowc(&c, tmp.begin(), tmp.end());
			it->userdata = (void *)(unsigned long)c;
		}
		else if ((it->ruleid == "mchar" || it->ruleid == "classmchar"))
		{
			wchar_t c = (wchar_t)(unsigned long)leveltree_firstchild(it, begin, end)->userdata;
			it->userdata = Char(c);
		}
		else if (it->ruleid == "anychar")
		{
			it->userdata = Any();
		}
		else if (it->ruleid == "hexnum")
		{
			std::string tmp = escaped(it->begin, it->end);
			if (tmp[0] != '0')
				tmp.insert(tmp.begin(), '0');
			it->userdata = (void *)std::stoul(tmp, nullptr, 16);
		}
		else if (it->ruleid == "decnum")
		{
			std::string c = escaped(it->begin, it->end);
			it->userdata = (void *)std::stoul(c, nullptr, 10);
		}
		else if ((it->ruleid == "mrange" || it->ruleid == "classrange"))
		{
			CIterator it_c1 = leveltree_firstchild(it, begin, end);
			CIterator it_c2 = leveltree_next(it_c1, begin, end);
			wchar_t c1 = (wchar_t)(unsigned long)it_c1->userdata;
			wchar_t c2 = (wchar_t)(unsigned long)it_c2->userdata;
			it->userdata = Range(c1, c2);
		}
		else if (it->ruleid == "mcharclass")
		{
			if (leveltree_nchildren(it, begin, end) > 1)
			{
				COr<parser<Iterator>> *op = (COr<parser<Iterator>> *)Or();
				it->userdata = op;
				for (
					CIterator it_child = leveltree_firstchild(it, begin, end);
					it_child != end;
					it_child = leveltree_next(it_child, begin, end))
					op->children_.push_back((MatchType *)it_child->userdata);
			}
			else
			{
				it->userdata = leveltree_firstchild(it, begin, end)->userdata;
			}
		}
		else if (it->ruleid == "ncharclass")
		{
			if (leveltree_nchildren(it, begin, end) > 1)
			{
				COr<parser<Iterator>> *op = (COr<parser<Iterator>> *)Or();
				COr<parser<Iterator>> *notop = (COr<parser<Iterator>> *)Not(op);
				it->userdata = notop;
				for (
					CIterator it_child = leveltree_firstchild(it, begin, end);
					it_child != end;
					it_child = leveltree_next(it_child, begin, end))
					op->children_.push_back((MatchType *)it_child->userdata);
			}
			else
			{
				it->userdata = Not((MatchType *)leveltree_firstchild(it, begin, end)->userdata);
			}
		}
		else if (it->ruleid == "mstr")
		{
			CIterator it_child = leveltree_firstchild(it, begin, end);
			it->userdata = Str(escaped(it_child->begin, it_child->end));
		}
		else if (it->ruleid == "call")
		{
			CIterator it_child = leveltree_firstchild(it, begin, end);
			it->userdata = Call(escaped(it_child->begin, it_child->end));
		}
		else if (it->ruleid == "skipon")
		{
			AutoSkipOn((MatchType *)leveltree_firstchild(it, begin, end)->userdata);
		}
		else if (it->ruleid == "skipoff")
		{
			AutoSkipOff();
		}
		else if (it->ruleid == "opt")
		{
			it->userdata = Opt((MatchType *)leveltree_firstchild(it, begin, end)->userdata);
		}
		else if (it->ruleid == "mul")
		{
			it->userdata = Mul((MatchType *)leveltree_firstchild(it, begin, end)->userdata);
		}
		else if (it->ruleid == "mulopt")
		{
			it->userdata = MulOpt((MatchType *)leveltree_firstchild(it, begin, end)->userdata);
		}
		else if (it->ruleid == "not")
		{
			it->userdata = Not((MatchType *)leveltree_firstchild(it, begin, end)->userdata);
		}
		else if (it->ruleid == "linestart")
		{
			it->userdata = LineStart((MatchType *)leveltree_firstchild(it, begin, end)->userdata);
		}
		else if (it->ruleid == "lineend")
		{
			it->userdata = LineEnd((MatchType *)leveltree_firstchild(it, begin, end)->userdata);
			// it->userdata = LineEnd();
		}
		else if (it->ruleid == "cat")
		{
			CCat<parser<Iterator>> *op = (CCat<parser<Iterator>> *)Cat();
			it->userdata = op;
			for (
				CIterator it_child = leveltree_firstchild(it, begin, end);
				it_child != end;
				it_child = leveltree_next(it_child, begin, end))
				op->children_.push_back((MatchType *)it_child->userdata);
		}
		else if (it->ruleid == "and")
		{
			CAnd<parser<Iterator>> *op = (CAnd<parser<Iterator>> *)And();
			it->userdata = op;
			for (
				CIterator it_child = leveltree_firstchild(it, begin, end);
				it_child != end;
				it_child = leveltree_next(it_child, begin, end))
				op->children_.push_back((MatchType *)it_child->userdata);
		}
		else if (it->ruleid == "or")
		{
			COr<parser<Iterator>> *op = (COr<parser<Iterator>> *)Or();
			it->userdata = op;
			for (
				CIterator it_child = leveltree_firstchild(it, begin, end);
				it_child != end;
				it_child = leveltree_next(it_child, begin, end))
				op->children_.push_back((MatchType *)it_child->userdata);
		}
		else if (it->ruleid == "sub")
		{
			CIterator it_c1 = leveltree_firstchild(it, begin, end);
			CIterator it_c2 = leveltree_next(it_c1, begin, end);
			MatchType *notop = Not((MatchType *)it_c2->userdata);
			MatchType *andop = And((MatchType *)it_c1->userdata, notop);
			it->userdata = andop;
		}
		else if (it->ruleid == "throw")
		{
			CIterator it_c1 = leveltree_firstchild(it, begin, end);
			CIterator it_c2 = leveltree_next(it_c1, begin, end);
			std::string msg(it_c2->begin, it_c2->end);
			it->userdata = Throw((MatchType *)it_c1->userdata, msg);
		}
		else if (it->ruleid == "rule" || it->ruleid == "rulen")
		{
			CIterator it_c1 = leveltree_firstchild(it, begin, end);
			CIterator it_c2 = leveltree_next(it_c1, begin, end);
			std::string rulename(it_c1->begin, it_c1->end);
			if (it->ruleid == "rule")
				it->userdata = (void *)Rule(rulename, (MatchType *)it_c2->userdata);
			else
				it->userdata = (void *)RuleN(rulename, (MatchType *)it_c2->userdata);
		}
	}

	template <typename Iterator>
	template <typename BNFIT>
	void parser<Iterator>::compile(BNFIT begin, BNFIT end)
	{
		EBNF_Compiler<BNFIT> ebnf_parser;

		size_t len = 0;
		for (BNFIT it = begin; it != end; it += len)
		{
			typename EBNF_Compiler<BNFIT>::parse_result_ex res = ebnf_parser.parse_extended("rules", it, begin, end);
			if (!res.status)
			{
				/*
				* Handle compile error
				*/
				throw parse_exception({res.error.msg, res.error.line, res.error.column, res.error.offset});
			}
			len = res.length;

			auto rit = res.ast.begin();
			while (rit != res.ast.end())
			{
				rit = cst::leveltree_walkpost(rit, res.ast.begin(), res.ast.end(),
						[&](typename std::vector<PRT_EntryEx<BNFIT>>::iterator rec,
							typename std::vector<PRT_EntryEx<BNFIT>>::iterator begin,
							typename std::vector<PRT_EntryEx<BNFIT>>::iterator end)
						{
							compile_entry(rec, begin, end);
						});
			}
		}
	}

	template <typename Iterator>
	void parser<Iterator>::compile(const std::string &bnf)
	{
		compile(bnf.begin(), bnf.end());
	}

	template <typename Iterator>
	void parser<Iterator>::compile(const char* bnf)
	{
		compile(bnf, bnf + strlen(bnf));
	}


} // namespace cst

#endif
