#ifndef _EBNFCOMPILER_H_
#define _EBNFCOMPILER_H_

#include "parser.h"

namespace cst {


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


	template<typename T>
	class EBNF_Compiler : public parser<T>
	{
	public:
		using CST_ParserType = parser<T>;
		using CST_ParserType::Iterator;
		using StackType = std::vector<PRT_Entry<T>>;
		using CST_ParserType::rules_;
		using CST_ParserType::RuleN;
		using CST_ParserType::Rule;
		using CST_ParserType::MulOpt;
		using CST_ParserType::Opt;
		using CST_ParserType::Mul;
		using CST_ParserType::Call;
		using CST_ParserType::Cat;
		using CST_ParserType::And;
		using CST_ParserType::Or;
		using CST_ParserType::Not;
		using CST_ParserType::Throw;
		using CST_ParserType::Char;
		using CST_ParserType::Str;
		using CST_ParserType::AutoSkipOn;
		using CST_ParserType::Range;
		using CST_ParserType::Any;
		using CST_ParserType::link;

		EBNF_Compiler() : parser<T>()
		{
			/*
			* s			::= [ \t\n]*
			*/
			RuleN("s", MulOpt(Or(Char(' '), Char('\t'), Char('\r'), Char('\n'), Call("comment"))));

			/*
			* b			::= [ \t]*
			*/
			RuleN("b", MulOpt(Or(Char(' '), Char('\t'))));

			/*
			* anychar		:= '.'
			*/
			Rule("anychar", Char('.'));

			/*
			* char			:= ('\\' .)| .
			*/
			Rule("char", Or(Cat(Char('\\'), Any()), Any()));

			/*
			* hexnum		:= '0'? ('x'|'X') ([0-9] | [a-f] | [A-F])+
			*/
			Rule("hexnum", Cat(Opt(Char('0')), Or(Char('x'), Char('X')), Mul(Or(Range('0', '9'), Range('a', 'f'), Range('A', 'F')))));

			/*
			* decnum		:= [0-9]+
			*/
			Rule("decnum", Mul(Range('0', '9')));

			/*
			* classrange	:= <char> '-' <char>
			*/
			Rule("classrange", Cat(Call("char"), Char('-'), Call("char")));

			/*
			* classmchar	:= <char>
			*/
			Rule("classmchar", Call("char"));

			/*
			* mcharclass	:= '[' (!('[' | ']' | '^') & (<classrange> | <classmchar>) )+  ']'
			*/
			Rule("mcharclass", Cat(Char('['), Mul(And(Not(Or(Char('['), Char(']'), Char('^'))), Or(Call("classrange"), Call("classmchar")))), Char(']')));

			/*
			* ncharclass	:= '[' '^' (!('[' | ']' | '^') & (<classrange> | <classmchar>) )+  ']'
			*/
			Rule("ncharclass", Cat(Char('['), Char('^'), Mul(And(Not(Or(Char('['), Char(']'), Char('^'))), Or(Call("classrange"), Call("classmchar")))), Char(']')));

			/*
			* mhexrange	::= '[' '#' <hexnum> '-' '#' <hexnum> ']'
			*/
			RuleN("mhexrange", Cat(Char('['), Char('#'), Call("hexnum"), Char('-'), Char('#'), Call("hexnum"), Char(']')));

			/*
			* mdecrange	::= '[' '#' <decnum> '-' '#' <decnum> ']'
			*/
			RuleN("mdecrange", Cat(Char('['), Char('#'), Call("decnum"), Char('-'), Char('#'), Call("decnum"), Char(']')));

			/*
			* mhex		::= '#' <hexnum> | '[' '#' <hexnum> ']'
			*/
			RuleN("mhex", Or(Cat(Char('#'), Call("hexnum")), Cat(Char('['), Char('#'), Call("hexnum"), Char(']'))));

			/*
			* mdec		::= '#' <decnum> | '[' '#' <decnum> ']'
			*/
			RuleN("mdec", Or(Cat(Char('#'), Call("decnum")), Cat(Char('['), Char('#'), Call("decnum"), Char(']'))));

			/*
			* mqchar		::= '\'' (!'\'' & <char>) '\''
			*/
			RuleN("mqchar", Cat(Char('\''), And(Not(Char('\'')), Call("char")), Char('\'')));

			/*
			* mrange		:= <mhexrange> | <mdecrange>
			*/
			Rule("mrange", Or(Call("mhexrange"), Call("mdecrange")));

			/*
			* rangeorclass::= '[' & throw(<mrange> | <mhex> | <mdec> | <ncharclass> | <mcharclass>, "Unexpected character")
			*/
			RuleN("rangeorclass", And(Char('['), Throw(Or(Call("mrange"), Call("mhex"), Call("mdec"), Call("ncharclass"), Call("mcharclass")), "Unexpected character")));

			/*
			* mchar		:= <mqchar> | <mhex> | <mdec>
			*/
			Rule("mchar", Or(Call("mqchar"), Call("mhex"), Call("mdec")));

			/*
			* str			:= ('\\' '\"' | !'"')+
			*/
			Rule("str", Mul(Or(Cat(Char('\\'), Char('\"')), Not(Char('\"')))));

			/*
			* sqstr		:= (('\\' '\'') | !'\'') (('\\' '\'') | !'\'')+
			*/
			Rule("sqstr", Cat(Or(Cat(Char('\\'), Char('\'')), Not(Char('\''))), Mul(Or(Cat(Char('\\'), Char('\'')), Not(Char('\''))))));

			/*
			* mstr		:= '"' <str> '"' | '\'' <sqstr> '\''
			*/
			Rule("mstr", Or(Cat(Char('\"'), Call("str"), Char('\"')), Cat(Char('\''), Call("sqstr"), Char('\''))));

			/*
			* call			:=	('<' & throw('<' <rulename> '>', "Invalid rule name")) | <rulename>
			*/
			Rule("call", Or(And(Char('<'), Throw(Cat(Char('<'), Call("rulename"), Char('>')), "Invalid rule name")), Call("rulename")));

			/*
			* string		::=	'"' <str> '"'
			*/
			RuleN("string", Cat(Char('\"'), Call("str"), Char('\"')));

			/*
			* rulename		:=	[a-zA-Z_][a-zA-Z0-9_]*
			*/
			Rule("rulename", Cat(Or(Range('a', 'z'), Range('A', 'Z'), Char('_')), MulOpt(Or(Range('a', 'z'), Range('A', 'Z'), Range('0', '9'), Char('_')))));

			/*
			* skipon(<b>);
			*/
			AutoSkipOn(Call("b"));

			/*
			* throw		:= "throw" '(' <expr> ',' <string> ')'
			*/
			Rule("throw", Cat(Str("throw"), Char('('), Call("expr"), Char(','), Call("string"), Char(')')));

			/*
			* comment		::= ('#' | "//") (!'\n')* '\n'
			*/
			RuleN("comment", Cat(Or(Char('#'), Cat(Char('/'), Char('/'))), MulOpt(Not(Char('\n'))), Char('\n')));

			/*
			* term			::= <mchar>
			*				|	<rangeorclass>
			*				|	<mstr>
			*				|	<throw>
			*				|	<anychar>
			*				|	<call>
			*				|	'(' & throw('(' <expr> ')', "Unmatched '('")
			*/
			RuleN("term", Or(Call("mchar"), Call("rangeorclass"), Call("mstr"), Call("throw"), Call("anychar"), Call("call"),
					And(Char('('), Throw(Cat(Char('('), Call("expr"), Char(')')),"Unmatched '('"))));

			/*
			* opt			:= 	<term> '?'
			*/
			Rule("opt", Cat(Call("term"), Char('?')));

			/*
			* mul			:=	<term> '+'
			*/
			Rule("mul", Cat(Call("term"), Char('+')));

			/*
			* mulopt		:=	<term> '*'
			*/
			Rule("mulopt", Cat(Call("term"), Char('*')));

			/*
			* not			:=	'!' <dterm>
			*/
			Rule("not", Cat(Char('!'), Call("dterm")));

			/*
			* linestart	:=	'^' <dterm>
			*/
			Rule("linestart", Cat(Char('^'), Call("dterm")));


			/*
			* lineend	:=	<dterm> '$'
			*/
			Rule("lineend", Cat(Call("dterm"), Char('$')));

			/*
			* dterm		::=	<not>
			* 				|	<linestart>
			* 				|	<lineend>
			*				|	<opt>
			*				|	<mul>
			*				|	<mulopt>
			*				|	<term>
			*/
			RuleN("dterm", Or(Call("not"), Call("linestart"), Call("lineend"), Call("opt"), Call("mul"), Call("mulopt"), Call("term")));

			/*
			* cat			:=	<dterm> (<b> <dterm>)+
			*/
			Rule("cat", Cat(Call("dterm"), Mul(Cat(Call("b"), Call("dterm")))));

			/*
			* andop		::=	<s> '&' <s>
			*/
			RuleN("andop", Cat(Call("s"), Char('&'), Call("s")));

			/*
			* orop			::=	<s> '|' <s>
			*/
			RuleN("orop", Cat(Call("s"), Char('|'), Call("s")));

			/*
			* subop		::=	<s> '-' <s>
			*/
			RuleN("subop", Cat(Call("s"), Char('-'), Call("s")));

			/*
			* and			:= 	(<cat> | <dterm>) ( <andop> (<cat> | <dterm>))+
			*/
			Rule("and", Cat(Or(Call("cat"), Call("dterm")), Mul(Cat(Call("andop"), Or(Call("cat"), Call("dterm"))))));

			/*
			* or			:= 	(<and> | <cat> | <dterm>) ( <orop> (<and> | <cat> | <dterm>))+
			*/
			Rule("or", Cat(Or(Call("and"), Call("cat"), Call("dterm")), Mul(Cat(Call("orop"), Or(Call("and"), Call("cat"), Call("dterm"))))));

			/*
			* sub			:= 	(<or> | <and> | <cat> | <dterm>) <subop> (<or> | <and> | <cat> | <dterm>)
			*/
			Rule("sub", Cat(Or(Call("or"), Call("and"), Call("cat"), Call("dterm")), Call("subop"), Or(Call("or"), Call("and"), Call("cat"), Call("dterm"))));

			/*
			* expr			::=	<sub>
			* 				|	<or>
			*				|	<and>
			*				|	<cat>
			*				|	<dterm>
			*/
			RuleN("expr", Or(Call("sub"), Call("or"), Call("and"), Call("cat"), Call("dterm")));

			/*
			* ruleop		::=	<s> (throw(':' ':'? '=', "Rule definition must use '::=' or ':='") & ":=") <s>
			*/
			RuleN("ruleop", Cat(Call("s"), And(Throw(Cat(Char(':'), Opt(Char(':')), Char('=')), "Rule definition must use '::=' or ':='"), Str(":=")), Call("s")));
			/*
			* rulenop		::=	<s> (throw(':' ':'? '=', "Rule definition must use '::=' or ':='") & "::=") <s>
			*/
			RuleN("rulenop", Cat(Call("s"), And(Throw(Cat(Char(':'), Opt(Char(':')), Char('=')), "Rule definition must use '::=' or ':='"), Str("::=")), Call("s")));

			/*
			* rule			:=	<rulename> <ruleop> throw(<expr>, "Invalid rule definition") <s> ';'?
			*/
			Rule("rule",  Cat(Call("rulename"), Call("ruleop"), Throw(Call("expr"), "Invalid rule definition"), Call("s"), Opt(Char(';'))));

			/*
			* rulen		:=	<rulename> <rulenop> throw(<expr>, "Invalid rule definition") <s> ';'?
			*/
			Rule("rulen",  Cat(Call("rulename"), Call("rulenop"), Throw(Call("expr"), "Invalid rule definition"), Call("s"), Opt(Char(';'))));

			/*
			* skipon		:=	"skipon" '(' <expr> ')' <s> ';'?
			*/
			Rule("skipon", Cat(Str("skipon"), Char('('), Call("expr"), Char(')'), Call("s"), Opt(Char(';'))));

			/*
			* skipoff		:=	"skipoff" '(' ')' <s> ';'?
			*/
			Rule("skipoff", Cat(Str("skipoff"), Char('('), Char(')'), Call("s"), Opt(Char(';'))));

			/*
			* bnfrule		::=	<skipon>
			*				|	<skipoff>
			*				|	<rule>
			*				|	<rulen>
			*/
			RuleN("bnfrule", Or(Call("skipon"), Call("skipoff"), Call("rule"), Call("rulen")));

			/*
			* rules		::= (<s> (. & throw(<bnfrule>, "Invalid rule")) <s>)
			*/
			RuleN("rules", Cat(Call("s"), And(Any(), Throw(Call("bnfrule"), "Invalid rule")), Call("s")));
			link();
		}

	};

}

#endif //_EBNFCOMPILER_H_
