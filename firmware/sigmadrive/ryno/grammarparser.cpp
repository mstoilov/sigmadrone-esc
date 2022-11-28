const char *g_grammar = R"grammar(   // Language Grammer
		/// Start of the grammer rules

		comment				::= ('/*' (!'*/')* '*/') | ('//' (!.$)+ .$)
		s					::= (' ' | '\t' | '\r' | '\n' | comment)+

		dec					::=	[0-9]+
		decimal				:=	dec
		hex					:=	[0-9a-fA-F]+
		bin					:=	[0-1]+
		float				:=	(dec '.' dec | '.' dec )
		sqstring			::=	'\'' (!'\'')* '\''
		dqstring			::=	'"' (!'"')* '"'
		alpha				::= [a-zA-Z_]
		alphanum			::= alpha | [0-9]
		idstr				::=	alpha alphanum*
		idname				:=  idstr
		skipon(s)
		identifier			::= idname ('[' expression ']')*
		skipoff()
		idref				:= identifier

		///
		/// Numbers
		///
		numconst 			::=	('0'? [bB] bin) | ('0'? [xX] hex) | float | decimal

		///
		/// Strings
		///
		stringconst			:=	sqstring | dqstring


		///
		/// Booleans
		///
		boolconst			:=	'true' | 'false'


		skipon(s)

		///
		/// Array
		///
		array				:= '[' (arrayelem (',' arrayelem)* ','?)? ']'
		arrayelem			:= expression

		///
		/// Map
		///
		map					:= '{' (mapelem (',' mapelem)* ','?)? '}'
		mapelem				:= expression ':' expression


		///
		/// Assignment
		///
		assignexpr 			:=  identifier '=' expression

		///
		/// Functin call
		///
		call				:= idref '(' expression? (',' expression s?)* ')'

		///
		/// Identifiers
		///
		identifierexpr		::= call | idref

		///
		/// Unary Expressions
		///
		preinc				:= '++' idref
		postinc				:= idref '++'
		predec				:= '--' idref
		postdec				:= idref '--'
		unary				::= preinc | postinc | predec | postdec

		///
		/// Expressions
		///
		return				:=  'return' toplevelexpr | 'return'
		break				:=  'break'
		continue			:=  'continue'
		primary				::=	anonfunctionblock | numconst | boolconst | assignexpr | call | unary | identifierexpr | parenexpr | stringconst | array | map
		parenexpr 			::= '(' expression ')'
		negterm				:=  '-' term
		posterm				::= '+' term
		term				::= primary | negterm | posterm
		mul					:= 	fact '*' term
		div					:= 	fact '/' term
		fact				::= mul | div | term
		leftshift			:= 	shift '<<' fact
		rightshift			:= 	shift '>>' fact
		shift				::= leftshift | rightshift | fact
		add					:= 	calcexpr '+' shift
		sub					:= 	calcexpr '-' shift
		calcexpr			::= (add | sub | shift | fact)
		lt					:=  calcexpr '<' calcexpr
		le					:=  calcexpr '<=' calcexpr
		gt					:=  calcexpr '>' calcexpr
		ge					:=  calcexpr '>=' calcexpr
		eq					:=  calcexpr '==' calcexpr
		ne					:=  calcexpr '!=' calcexpr
		logicexpr			::= eq | ne | lt | le | gt | ge
		expression			::= logicexpr | calcexpr
		toplevelexpr		:=	expression
		topassignexpr		:=	assignexpr
		expressionend		::= ';'
		allstatements		::= break | continue | return | topassignexpr | toplevelexpr
		expression_statement	::=	expressionend | allstatements expressionend

		///
		/// Control Flow
		///
		body					::= '{' statement* '}' | statement
		if_body				:=  body
		if_cond				:=  expression
		if_statement			:=  'if' '(' if_cond ')' if_body ('else' 'if' '(' if_cond ')' if_body)* ('else' if_body)?
		while_cond			:=  expression
		while_body			:=  body		
		while_statement		:=  'while' '(' while_cond ')' while_body
		for_init				:=  toplevelexpr?
		for_cond				:=  expression?
		for_postaction		:=  toplevelexpr?
		for_body				:=  body
		for_statement		:=  'for' '(' for_init ';' for_cond ';' for_postaction ')' for_body

		funcname				:= idstr
		funcarg					:= idstr
		paramlist				:= funcarg (',' funcarg)*
		functionblock			:= function
		function_body			:= '{' statement* '}'
		function				:= 'function' funcname '(' paramlist? ')' function_body
		anonfunction			:= 'function' '(' paramlist? ')' function_body
		anonfunctionblock		:= anonfunction

		statement				:=	if_statement 
								|	functionblock
								|	while_statement
								|	for_statement
								|	expression_statement

		fragment			:= (s? statement | s )

		skipoff()
		/// End of the grammer rules
)grammar";

#include "grammarparser.h"

namespace ryno {


GrammarParser::GrammarParser() : base()
{
	Initialize();
}

void GrammarParser::Initialize()
{
#ifdef USE_RYNO_BNF
	compile(g_grammar);

#else

	// comment				::= ('/*' (!'*/')* '*/') | ('//' (!.$)+ .$)
	RuleN("comment", Or(Cat(Str("/*"), MulOpt(Not(Str("*/"))), Str("*/")), Cat(Str("//"), Mul(Not(LineEnd(Any()))), LineEnd(Any()))));

	// s					::= ([ \t\r\n] | comment)+
	RuleN("s", Mul(Or(Char(' '), Char('\t'), Char('\r'), Char('\n'), Call("comment"))));

	// dec					:=	[0-9]+
	RuleN("dec", Mul(Range('0', '9')));

	// decimal				:=	dec
	Rule("decimal", Call("dec"));

	// hex					:=	[0-9a-fA-F]+
	Rule("hex", Mul(Or(Range('0', '9'), Range('a', 'f'), Range('A', 'F'))));

	// bin					:=	[01]+
	Rule("bin", Mul(Range('0', '1')));

	// float				:=	(dec '.' dec | '.' dec )
	Rule("float", Or(Cat(Call("dec"), Char('.'), Call("dec")), Cat(Char('.'), Call("dec"))));

	// sqstring				::=	'\'' (!'\'')* '\''
	RuleN("sqstring", Cat(Char('\''), MulOpt(Not(Char('\''))), Char('\'')));

	// dqstring				::=	'"' (!'"')* '"'
	RuleN("dqstring", Cat(Char('"'), MulOpt(Not(Char('"'))), Char('"')));

	// alpha				::= [a-zA-Z_]
	RuleN("alpha", Or(Range('a', 'z'), Range('A', 'Z'), Char('_')));

	// alphanum				::= alpha | [0-9]
	RuleN("alphanum", Or(Call("alpha"), Range('0', '9')));

	// idstr				::=	alpha alphanum*
	RuleN("idstr", Cat(Call("alpha"), MulOpt(Call("alphanum"))));

	// idname				:=  idstr
	Rule("idname", Call("idstr"));

	// skipon(s)
	AutoSkipOn(Call("s"));

	// identifier			::= idname ('[' expression ']')*
	RuleN("identifier", Cat(Call("idname"), MulOpt(Cat(Char('['), Call("expression"), Char(']')))));

	// skipoff()
	AutoSkipOff();

	// idref				:= identifier
	Rule("idref", Call("identifier"));

	// numconst 			::=	('0'? [bB] bin) | ('0'? [xX] hex) | float | decimal
	RuleN("numconst",	Or(Cat(Opt(Char('0')), Or(Char('b'), Char('B')), Call("bin")) , 
							Cat(Opt(Char('0')), Or(Char('x'), Char('X')), Call("hex")),
							Call("float"), 
							Call("decimal")));

	// stringconst			:=	sqstring | dqstring
	Rule("stringconst",	Or(Call("sqstring"), Call("dqstring")));

	// boolconst			:=	'true' | 'false'
	Rule("boolconst", Or(Str("true"), Str("false")));

	// skipon(s)
	AutoSkipOn(Call("s"));

	// array				:= '[' (arrayelem (',' arrayelem)* ','?)? ']'
	Rule("array", Cat(Char('['), Opt(Cat(Call("arrayelem"), MulOpt(Cat(Char(','), Call("arrayelem"))), Opt(Char(',')))), Char(']')));

	// arrayelem			:= expression
	Rule("arrayelem", Call("expression"));

	// map					:= '{' (mapelem (',' mapelem)* ','?)? '}'
	Rule("map", Cat(Char('{'), Opt(Cat(Call("mapelem"), MulOpt(Cat(Char(','), Call("mapelem"))), Opt(Char(',')))), Char('}')));


	// mapelem				:= expression ':' expression
	Rule("mapelem", Cat(Call("expression"), Char(':'), Call("expression")));

	// assignexpr 			:=  identifier '=' expression
	Rule("assignexpr", Cat(Call("identifier"), Char('='), Call("expression")));

	///
	/// Functin call
	///
	// call				:= idref '(' expression? (',' expression s?)* ')'
	Rule("call", Cat(Call("idref"), Char('('), Opt(Call("expression")), MulOpt(Cat(Char(','), Call("expression"), Opt(Call("s")))), Char(')')));


	///
	/// Identifiers
	///
	// identifierexpr		::= call | idref
	RuleN("identifierexpr", Or(Call("call"), Call("idref")));

	// preinc				:= '++' idref
	Rule("preinc", Cat(Str("++"), Call("idref")));

	// postinc				:= idref '++'
	Rule("postinc", Cat(Call("idref"), Str("++")));

	// predec				:= '--' idref
	Rule("predec", Cat(Str("--"), Call("idref")));

	// postdec				:= idref '--'
	Rule("postdec", Cat(Call("idref"), Str("--")));

	// unary				::= preinc | postinc | predec | postdec
	RuleN("unary", Or(Call("preinc"), Call("postinc"), Call("predec"), Call("postdec")));

	// return				:=  'return' toplevelexpr | 'return'
	Rule("return", Or(Cat(Str("return"), Call("toplevelexpr")), Str("return")));

	// break				:=  'break'
	Rule("break", Str("break"));

	// continue			:=  'continue'
	Rule("continue", Str("continue"));

	// primary				::=	anonfunctionblock | numconst | boolconst | assignexpr | call | unary | identifierexpr | parenexpr | stringconst | array | map
	RuleN("primary", Or(Call("anonfunctionblock"),
						Call("numconst"),
						Call("boolconst"),
						Call("assignexpr"),
						Call("call"),
						Call("unary"),
						Call("identifierexpr"),
						Call("parenexpr"),
						Call("stringconst"),
						Call("array"),
						Call("map")));

	// parenexpr 			::= '(' expression ')'
	RuleN("parenexpr", Cat(Char('('), Call("expression"), Char(')')));

	// negterm				:=  '-' term
	Rule("negterm", Cat(Char('-'), Call("term")));

	// posterm				::= '+' term
	RuleN("posterm", Cat(Char('+'), Call("term")));

	// term					::= primary | negterm | posterm
	RuleN("term", Or(Call("primary"), Call("negterm"), Call("posterm")));

	// mul					:= 	fact '*' term
	Rule("mul", Cat(Call("fact"), Char('*'), Call("term")));

	// div					:= 	fact '/' term
	Rule("div", Cat(Call("fact"), Char('/'), Call("term")));

	// fact					::= mul | div | term
	RuleN("fact", Or(Call("mul"), Call("div"), Call("term")));

	// leftshift			:= 	shift '<<' fact
	Rule("leftshift", Cat(Call("shift"), Str("<<"), Call("fact")));

	// rightshift			:= 	shift '>>' fact
	Rule("rightshift", Cat(Call("shift"), Str(">>"), Call("fact")));

	// shift				::= leftshift | rightshift | fact
	RuleN("shift", Or(Call("leftshift"), Call("rightshift"), Call("fact")));

	// add					:= 	calcexpr '+' shift
	Rule("add", Cat(Call("calcexpr"), Char('+'), Call("shift")));

	// sub					:= 	calcexpr '-' shift
	Rule("sub", Cat(Call("calcexpr"), Char('-'), Call("shift")));

	// calcexpr				::= (add | sub | shift | fact)
	RuleN("calcexpr", Or(Call("add"), Call("sub"), Call("shift"), Call("fact")));

	// lt					:=  calcexpr '<' calcexpr
	Rule("lt", Cat(Call("calcexpr"), Char('<'), Call("calcexpr")));

	// le					:=  calcexpr '<=' calcexpr
	Rule("le", Cat(Call("calcexpr"), Str("<="), Call("calcexpr")));

	// gt					:=  calcexpr '>' calcexpr
	Rule("gt", Cat(Call("calcexpr"), Char('>'), Call("calcexpr")));

	// ge					:=  calcexpr '>=' calcexpr
	Rule("ge", Cat(Call("calcexpr"), Str(">="), Call("calcexpr")));

	// eq					:=  calcexpr '==' calcexpr
	Rule("eq", Cat(Call("calcexpr"), Str("=="), Call("calcexpr")));

	// ne					:=  calcexpr '!=' calcexpr
	Rule("ne", Cat(Call("calcexpr"), Str("!="), Call("calcexpr")));

	// logicexpr			::= eq | ne | lt | le | gt | ge
	RuleN("logicexpr", Or(Call("eq"),
						Call("ne"),
						Call("lt"),
						Call("le"),
						Call("gt"),
						Call("ge")));

	// expression			::= logicexpr | calcexpr
	RuleN("expression", Or(Call("logicexpr"),
						Call("calcexpr")));

	// topassignexpr		:=	assignexpr
	Rule("topassignexpr", Call("assignexpr"));

	// toplevelexpr			:=	expression
	Rule("toplevelexpr", Call("expression"));

	// expressionend		::= ';'
	RuleN("expressionend", Char(';'));

	// allstatements		::= break | continue | return | toplevelexpr
	RuleN("allstatements", Or(Call("break"), Call("continue"), Call("return"), Call("topassignexpr"), Call("toplevelexpr")));

	// expression_statement	::=	expressionend | allstatements expressionend
	RuleN("expression_statement", Or(Call("expressionend"), Cat(Call("allstatements"), Call("expressionend"))));

	///
	/// Control Flow
	///
	// body					::= '{' statement* '}' | statement
	RuleN("body", Or(Cat(Char('{'), MulOpt(Call("statement")), Char('}')), Call("statement")));

	// if_body				:=  body
	Rule("if_body", Call("body"));

	// if_cond				:=  expression
	Rule("if_cond", Call("expression"));

	// if_statement			:=  'if' '(' if_cond ')' if_body ('else' 'if' '(' if_cond ')' if_body)* ('else' if_body)?
	Rule("if_statement", Cat(Str("if"), Char('('), Call("if_cond"), Char(')'), Call("if_body"), 
				MulOpt(Cat(Str("else"), Str("if"), Char('('), Call("if_cond"), Char(')'), Call("if_body"))), 
				Opt(Cat(Str("else"), Call("if_body")))));

	// while_cond			:=  expression
	Rule("while_cond", Call("expression"));

	// while_body			:=  body
	Rule("while_body", Call("body"));

	// while_statement		:=  'while' '(' while_cond ')' while_body
	Rule("while_statement", Cat(Str("while"), Char('('), Call("while_cond"), Char(')'), Call("while_body")));

	// for_init				:=  toplevelexpr?
	Rule("for_init", Opt(Call("toplevelexpr")));

	// for_cond				:=  expression?
	Rule("for_cond", Opt(Call("expression")));

	// for_postaction		:=  toplevelexpr?
	Rule("for_postaction", Opt(Call("toplevelexpr")));

	// for_body				:=  body
	Rule("for_body", Call("body"));

	// for_statement		:=  'for' '(' for_init ';' for_cond ';' for_postaction ')' for_body
	Rule("for_statement", 	Cat(Str("for"), 
							Char('('), 
							Call("for_init"), 
							Char(';'), 
							Call("for_cond"), 
							Char(';'), 
							Call("for_postaction"), 
							Char(')'), 
							Call("for_body")));


	// funcname				:= idstr
	Rule("funcname", Call("idstr"));

	// funcarg				:= idstr
	Rule("funcarg", Call("idstr"));

	// paramlist			:= funcarg (',' funcarg)*
	Rule("paramlist", Cat(Call("funcarg"), MulOpt(Cat(Char(','), Call("funcarg")))));


	// functionblock		:= function
	Rule("functionblock", Call("function"));

	// function_body		:= '{' statement* '}'
	Rule("function_body", Cat(Char('{'), MulOpt(Call("statement")), Char('}')));

	// function				:= 'function' funcname '(' paramlist? ')' function_body
	Rule("function", Cat(Str("function"),
						Call("funcname"),
						Char('('),
						Opt(Call("paramlist")),
						Char(')'),
						Call("function_body")));

	// anonfunction			:= 'function' '(' paramlist? ')' function_body
	Rule("anonfunction", Cat(Str("function"),
						Char('('),
						Opt(Call("paramlist")),
						Char(')'),
						Call("function_body")));

	// anonfunctionblock	:= anonfunction
	Rule("anonfunctionblock", Call("anonfunction"));

	// statement			:=	if_statement 
	// 						|	functionblock
	// 						|	while_statement
	// 						|	for_statement
	// 						|	expression_statement
	Rule("statement", Or(Call("if_statement"), 
						Call("functionblock"), 
						Call("while_statement"), 
						Call("for_statement"), 
						Call("expression_statement")));

	// fragment				:= s? statement | s
	Rule("fragment", Or(Cat(Opt(Call("s")), Call("statement")), Call("s")));

	// skipoff()
	AutoSkipOff();
#endif

	link();
}

}