#include "runtime.h"
#include "stdfunc.h"

namespace ryno {

RunTime::RunTime(size_t stacksize)
{
	vm_ = ry_cpu_create(stacksize, nullptr);
	CPU_CPUREG_SET(vm_, FP, 0UL);
	CPU_CPUREG_EMPLACE(vm_, LP, &globals_, true);
	CPU_CPUREG_EMPLACE(vm_, GP, &globals_, true);
	Initialize(*this);

}

RunTime::~RunTime()
{
	ry_cpu_destroy(vm_);
}

size_t RunTime::Compile(CodeFragment& code, std::string::const_iterator it, std::string::const_iterator begin, std::string::const_iterator end)
{
  size_t ret = compiler_.ParseFragment(code, it, begin, end);
  compiler_.CompileFragment(code);
  return ret;
}

varreg_t RunTime::Exec(const std::string& src, bool exec_debug)
{
	for (std::string::const_iterator it = src.begin(); it != src.end(); ) {
		CodeFragment code;
		it += Compile(code, it, src.begin(), src.end());
		code.Normalize();
		CPU_CPUREG_EMPLACE(vm_, CO, &code.objects_, true);
		ry_cpu_exec(vm_, code.CurrentBlock().data(), 0, exec_debug ? 1 : 0);
	}
	varreg_t ret = CPU_CPUREG_GET(vm_, R0);
	return ret;
}

void RunTime::AddSWI(const std::string& name, rycpu_swi_t swi)
{
	globals_.AssignAtPP(new StringObject(name), new IntObject((r_uint_t)swi));
}

void RunTime::SetVar(const std::string& name, RyObject* obj)
{
	globals_.AssignAtPP(new StringObject(name), obj);
}

RyObject* RunTime::GetVar(const std::string& name)
{
	StringObject rystr(name);
	return globals_.At(&rystr);
}

void RunTime::DumpAST(std::ostream& os, const std::string& in)
{
	AST ast = compiler_.Parse(in);
	for (auto it = ast.begin(); it != ast.end(); ) {
		it = cst::leveltree_walk(it, ast.begin(), ast.end(), 
			[&](AST::const_iterator it, AST::const_iterator, AST::const_iterator) { os << "    " << it->repr() << std::endl; },
			[](AST::const_iterator, AST::const_iterator, AST::const_iterator){});
	}
}

void RunTime::DumpCode(std::ostream& os, const std::string& in)
{
	size_t retsize = 0;
	for (std::string::const_iterator it = in.begin(); it != in.end(); it += retsize) {
		CodeFragment code;
		retsize = Compile(code, it, in.begin(), in.end());
		std::cout << " ------- " << code.name_ << "(";
		for (const auto& arg : code.arguments_)
			std::cout << arg << ", ";
		std::cout << ")";
		std::cout << " ------- " << std::endl;
		for (auto& b : code.basicblocks_)
			std::cout << b.repr() << std::endl;
		std::cout << std::endl;
	}
}

void RunTime::DumpRules(std::ostream& os)
{
	compiler_.dump_rules(os);
}

}
