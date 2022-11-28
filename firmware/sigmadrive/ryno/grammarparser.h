#ifndef _GRAMMARPARSER_H_
#define _GRAMMARPARSER_H_

#include "cst/parser.h"


namespace ryno {

class GrammarParser : public cst::parser<std::string::const_iterator> {
public:
	using base = cst::parser<std::string::const_iterator>;
	GrammarParser();
	void Initialize();
};


}

#endif
