#ifndef _MODULE_H_
#define _MODULE_H_

#include "rycpu.h"
#include "codefragment.h"
#include "cst/parser.h"
#include "ryobject.h"
#include "floatobject.h"
#include "intobject.h"
#include "arrayobject.h"
#include "stringobject.h"
#include "boolobject.h"
#include "mapobject.h"
#include "functionobject.h"
#include "smart_ptr.h"
#include "error.h"
#include "module.h"
#include "compiler.h"


namespace ryno {

class RunTime {
public:
	RunTime(size_t stacksize = 4096);
	~RunTime();
	varreg_t Exec(const std::string& src, bool exec_debug = false);
	void AddSWI(const std::string& name, rycpu_swi_t swi);
	void SetVar(const std::string& name, RyObject* obj);
	RyObject* GetVar(const std::string& name);
	
	size_t Compile(CodeFragment& code, std::string::const_iterator it, std::string::const_iterator begin, std::string::const_iterator end);
	void DumpAST(std::ostream& os, const std::string& in);
	void DumpCode(std::ostream& os, const std::string& in);
	void DumpRules(std::ostream& os);

protected:
	Compiler compiler_;
	MapObject globals_;
	rycpu_t* vm_;
};

}


#endif
