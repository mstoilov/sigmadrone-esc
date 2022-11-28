#include "runtime.h"

namespace ryno {

static void print(rycpu_t *cpu, asmins_t *ins)
{
	size_t params = ry_cpu_stackparam_count(cpu);
	for (size_t n = 0; n < params; n++) 
	{
		varreg_t& param = ry_cpu_stackparam_get(cpu, n);
		if (std::holds_alternative<r_uint_t>(param)) {
			std::cout << CPU_REG_GETU(param);
		} else {
			std::cout << std::get<RyPointer>(param)->ToStr() ;
		}
	}
	CPU_CPUREG_EMPLACE(cpu, R0, new IntObject(0));
}

static void printl(rycpu_t *cpu, asmins_t *ins)
{
	print(cpu, ins);
	std::cout << std::endl;
}

void Initialize(RunTime& rt)
{
	rt.AddSWI("print", ryno::print);
	rt.AddSWI("printl", ryno::printl);

}


}