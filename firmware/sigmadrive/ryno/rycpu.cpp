#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "rycpu.h"
#include "arrayobject.h"
#include "mapobject.h"
#include "intobject.h"
#include "floatobject.h"
#include "boolobject.h"
#include "stringobject.h"
#include "functionobject.h"

namespace ryno {

#undef CPU_OP_ENTRY
#undef CPU_OP_ENTRY_FIRST
#define CPU_OP_ENTRY(op, handler) #op,
#define CPU_OP_ENTRY_FIRST(op, handler) #op,

static const char* op_names[] = {
		CPU_OP_TABLE
};

#undef CPU_OP_ENTRY
#undef CPU_OP_ENTRY_FIRST
#define CPU_OP_ENTRY(op, handler) static void handler(rycpu_t *cpu, asmins_t *ins);
#define CPU_OP_ENTRY_FIRST(op, handler) static void handler(rycpu_t *cpu, asmins_t *ins);


CPU_OP_TABLE

#undef CPU_OP_ENTRY
#undef CPU_OP_ENTRY_FIRST
#define CPU_OP_ENTRY(op, handler) handler,
#define CPU_OP_ENTRY_FIRST(op, handler) handler,

static cpu_op_t op_handlers[] = {
	CPU_OP_TABLE
};


static int rycpu_asm_dump_pi_to_str(const asmins_t *pi, char *str, size_t size);
static int rycpu_asm_dump_reg_to_str(unsigned int reg, char *str, size_t size);
static void dumpregs(const rycpu_t *vm, const asmins_t *pi);


asmins_t ry_asmc(r_uint_t opcode, r_uint_t op1, r_uint_t op2, r_uint_t op3, r_uint_t data, r_uint_t cond)
{
	asmins_t a;

	memset(&a, 0, sizeof(a));
	a.opcode = (uint16_t) CPU_ASMINS_OPCODE(opcode);
	a.op1 = (uint16_t)op1;
	a.op2 = (uint16_t)op2;
	a.op3 = (uint16_t)op3;
	a.cond = (uint16_t)cond;
	a.data = data;
	if ((uint8_t)op1 == DA || (uint8_t)op2 == DA || (uint8_t)op3 == DA)
		a.da = 1;
	return a;
}

asmins_t ry_asm(r_uint_t opcode, r_uint_t op1, r_uint_t op2, r_uint_t op3, r_uint_t data)
{
	return ry_asmc(opcode, op1, op2, op3, data, 0);
}

rycpu_t *ry_cpu_create(r_uint_t stacksize, swi_table_t* switables)
{
	rycpu_t *cpu;

	cpu = new rycpu_t();
	if (!cpu)
		return ((rycpu_t*)0);
	cpu->switables = switables;
	cpu->stack.resize(stacksize);
	return cpu;
}

rycpu_t *ry_cpu_create_default()
{
	return ry_cpu_create(4096, NULL);
}

void ry_cpu_destroy(rycpu_t *cpu)
{
	delete cpu;
}

void ry_cpu_switables(rycpu_t *cpu, swi_table_t* switables)
{
	cpu->switables = switables;
}

int ry_cpu_exec(rycpu_t *cpu, asmins_t *prog, r_uint_t off, r_uint_t debug)
{
	asmins_t *pi;

	CPU_CPUREG_SETIP(cpu, PC, prog + off);
	cpu->abort = 0;
	cpu->error = 0;
	do {
		pi = CPU_CPUREG_GETIP(cpu, PC);
		if (pi->da) {
			CPU_CPUREG_SETU(cpu, DA, pi->data);
		}
#if CPU_CONDITIONAL_INSTRUCTIONS
		if (pi->cond) {
			switch (pi->cond) {
			case CPU_CEXEC_GRE:
				if (!((cpu->status & CPU_STATUS_N) == 0 && (cpu->status & CPU_STATUS_Z) == 0))
					goto skipexec;
				break;
			case CPU_CEXEC_GEQ:
				if (!((cpu->status & CPU_STATUS_N) == 0 || (cpu->status & CPU_STATUS_Z) == 1))
					goto skipexec;
				break;
			case CPU_CEXEC_EQ:
				if (!((cpu->status & CPU_STATUS_Z)))
					goto skipexec;
				break;
			case CPU_CEXEC_NEQ:
				if (!((cpu->status & CPU_STATUS_Z) == 0))
					goto skipexec;
				break;
			case CPU_CEXEC_LEQ:
				if (!((cpu->status & CPU_STATUS_N) || (cpu->status & CPU_STATUS_Z)))
					goto skipexec;
				break;
			case CPU_CEXEC_LES:
				if (!((cpu->status & CPU_STATUS_N)))
					goto skipexec;
				break;
			default:
				goto skipexec;
			};
		}
#endif
		op_handlers[pi->opcode](cpu, pi);
		if (debug) {
			dumpregs(cpu, pi);
		}
#if CPU_CONDITIONAL_INSTRUCTIONS
skipexec:
#endif
		CPU_CPUREG_INCIP(cpu, PC, 1);
	} while (!cpu->abort);
	if (cpu->error)
		return -1;
	return 0;
}

r_uint_t ry_cpu_swilookup(swi_table_t *table, const char* table_name, const char* swi_name)
{
	r_uint_t tid = 0;
	for (tid = 0; table && table->handlers; table++, tid++) {
		if (strcmp(table->name, table_name) == 0) {
			r_uint_t sid = 0;
			swi_handlers_t* handlers = table->handlers;
			for (sid = 0; handlers->op; handlers++, sid++) {
				if (strcmp(handlers->name, swi_name) == 0) {
					return (r_uint_t)CPU_SWI_ID(tid, sid);
				}
			}
		}
	}
	return (r_uint_t)-1;
}

void ryreg_set_signed(varreg_t *r, r_int_t val)
{
	CPU_REG_SETU(*r, (r_uint_t) val);
}

varreg_t ryreg_create_signed(r_int_t val)
{
	varreg_t r;
	ryreg_set_signed(&r, val);
	return r;
}

void ryreg_set_unsigned(varreg_t *r, r_uint_t val)
{
	CPU_REG_SETU(*r, (r_uint_t) val);
}

varreg_t ryreg_create_unsigned(r_uint_t val)
{
	varreg_t r;
	ryreg_set_signed(&r, val);
	return r;
}

void ryreg_set_pointer(varreg_t *r, r_pointer_t val)
{
	CPU_REG_SETP(*r, val);
}

varreg_t ryreg_create_pointer(r_pointer_t val)
{
	varreg_t r;
	ryreg_set_pointer(&r, val);
	return r;
}

void ry_cpu_push(rycpu_t* cpu, const varreg_t& reg)
{
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP) + 1;

	CPU_STACK_CHECKSIZE(cpu, sp);
	CPU_STACK_WRITE(cpu, sp, reg);
	CPU_CPUREG_SETU(cpu, SP, sp);
}

void ry_cpu_push(rycpu_t* cpu, varreg_t&& reg)
{
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP) + 1;

	CPU_STACK_CHECKSIZE(cpu, sp);
	CPU_STACK_WRITE(cpu, sp, std::move(reg));
	CPU_CPUREG_SETU(cpu, SP, sp);
}

void ry_cpu_pushref(rycpu_t* cpu, RyObject* obj)
{
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP) + 1;
	varreg_t reg(RyPointer(obj, true));

	CPU_STACK_CHECKSIZE(cpu, sp);
	CPU_STACK_WRITE(cpu, sp, std::move(reg));
	CPU_CPUREG_SETU(cpu, SP, sp);
}

void ry_cpu_pushref(rycpu_t* cpu, const RyPointer& obj)
{
	ry_cpu_pushref(cpu, obj.get());
}

varreg_t ry_cpu_pop(rycpu_t *cpu)
{
	varreg_t ret;
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP);
	if (sp == 0) {
		/*
		 * TBD:
		 * Error Handling is missing
		 */
		return ret;
	}
	ret = CPU_STACK_READ(cpu, sp);
	CPU_STACK_CLEAR(cpu, sp);
	CPU_CPUREG_SETU(cpu, SP, sp - 1);
	return ret;
}

size_t ry_cpu_stackparam_count(rycpu_t* cpu)
{
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP);
	r_uint_t fp = CPU_CPUREG_GETU(cpu, FP);
	/*
	 * Subtract 1, because the function name is also on the stack
	 */
	return sp - fp - RYO_CALL_SAVEDREGS - 1;
}

varreg_t& ry_cpu_stackparam_get(rycpu_t*cpu, size_t n)
{
	r_uint_t fp = CPU_CPUREG_GETU(cpu, FP);
	return CPU_STACK_READ(cpu, fp + n + 2);
}


std::string ry_cpu_dump(const asmins_t *pi, r_uint_t count)
{
	std::string ret;
	char buffer[512];
	while (count) {
		rycpu_asm_dump_pi_to_str(pi, buffer, sizeof(buffer));
		ret += "\n";
		ret += buffer;
		++pi;
		--count;
	}
	return ret;
}

static void print_varreg(const varreg_t& reg)
{
	if (std::holds_alternative<r_uint_t>(reg)) {
		std::cout << CPU_REG_GETU(reg) << ", ";
	} else {
		if (std::get<RyPointer>(reg).ref)
			std::cout << '&';
		std::cout << std::get<RyPointer>(reg)->Repr() << ", ";
	}
}

static void dumpregs(const rycpu_t *vm, const asmins_t *pi)
{
    int ret;
	char buffer[1024];

	ret = rycpu_asm_dump_pi_to_str(pi, buffer, sizeof(buffer));
	if (ret < 0)
		return;
    ret = snprintf(buffer + ret, sizeof(buffer) - ret, "                                                                                        ");
	buffer[45] = '\0';
	printf("%s", buffer);

	print_varreg(CPU_CPUREG_GET(vm, 0));
	print_varreg(CPU_CPUREG_GET(vm, 1));
	print_varreg(CPU_CPUREG_GET(vm, 2));
	print_varreg(CPU_CPUREG_GET(vm, 3));
	// print_varreg(CPU_CPUREG_GET(vm, 4));
	// print_varreg(CPU_CPUREG_GET(vm, 5));
	// print_varreg(CPU_CPUREG_GET(vm, 6));
	// print_varreg(CPU_CPUREG_GET(vm, 7));
	// print_varreg(CPU_CPUREG_GET(vm, 8));

	printf(" FP=%ld, SP=%ld, LR=0x%lx, PC=0x%lx, S( %c%c%c%c%c )",
		(long int)CPU_CPUREG_GETU(vm, FP),
		(long int)CPU_CPUREG_GETU(vm, SP),
   		(long int)CPU_CPUREG_GETU(vm, LR),
		(long int)CPU_CPUREG_GETU(vm, PC),
   		vm->status & CPU_STATUS_E ? 'E' : '_',
   		vm->status & CPU_STATUS_V ? 'V' : '_',
   		vm->status & CPU_STATUS_C ? 'C' : '_',
   		vm->status & CPU_STATUS_N ? 'N' : '_',
   		vm->status & CPU_STATUS_Z ? 'Z' : '_');
	printf("\n");
}

static int rycpu_asm_dump_pi_to_str(const asmins_t *pi, char *str, size_t size)
{
	int ret = 0, sz = size;

	if ((ret = snprintf(str, sz, "%12s   ", op_names[pi->opcode])) < 0)
		return ret;
	str += ret;
	sz -= ret;

	if ((ret = rycpu_asm_dump_reg_to_str(pi->op1, str, sz)) < 0)
		return ret;
	str += ret;
	sz -= ret;

	if ((ret = snprintf(str, sz, " ")) < 0)
		return ret;
	str += ret;
	sz -= ret;

	if ((ret = rycpu_asm_dump_reg_to_str(pi->op2, str, sz)) < 0)
		return ret;
	str += ret;
	sz -= ret;

	if ((ret = snprintf(str, sz, " ")) < 0)
		return ret;
	str += ret;
	sz -= ret;

	if ((ret = rycpu_asm_dump_reg_to_str(pi->op3, str, sz)) < 0)
		return ret;
	str += ret;
	sz -= ret;

	if ((ret = snprintf(str, sz, " ")) < 0)
		return ret;
	str += ret;
	sz -= ret;

	if ((ret = snprintf(str, sz, "0x%lx  ", (unsigned long)pi->data)) < 0)
		return ret;

	str += ret;
	sz -= ret;

	return size - sz;
}

static int rycpu_asm_dump_reg_to_str(unsigned int reg, char *str, size_t size)
{
	int ret = 0;

	if (reg == XX)
		ret = snprintf(str, size, "XX ");
	else if (reg == FP)
		ret = snprintf(str, size, "FP ");
	else if (reg == SP)
		ret = snprintf(str, size, "SP ");
	else if (reg == LR)
		ret = snprintf(str, size, "LR ");
	else if (reg == PC)
		ret = snprintf(str, size, "PC ");
	else if (reg == DA)
		ret = snprintf(str, size, "DA ");
	else
		ret = snprintf(str, size, "R%d ",  reg);

	return ret;
}


static void cpu_op_ext(rycpu_t *cpu, asmins_t *ins)
{
	cpu->abort = 1;
}

static void cpu_op_abort(rycpu_t *cpu, asmins_t *ins)
{
	CPU_ABORT(cpu, CPU_CPUREG_GETU(cpu, ins->op1));
}

static void cpu_op_swi(rycpu_t *cpu, asmins_t *ins)
{
	unsigned int ntable = (unsigned int) CPU_SWI_TABLE(CPU_CPUREG_GETU(cpu, ins->op1));
	unsigned int nswi = (unsigned int) CPU_SWI_NUM(CPU_CPUREG_GETU(cpu, ins->op1));

	if (!cpu->switables || CPU_CPUREG_GETU(cpu, ins->op1) == (r_uint_t)-1)
		CPU_ABORT(cpu, CPU_E_SWINUM);
	cpu->switables[ntable].handlers[nswi].op(cpu, ins);
}

static void cpu_op_prn(rycpu_t *cpu, asmins_t *ins)
{
#ifndef STM32F745xx
	printf("R%d = %lu(0x%lx)\n", ins->op1, CPU_CPUREG_GETU(cpu, ins->op1), CPU_CPUREG_GETU(cpu, ins->op1));
#endif
}

static void cpu_op_mov(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SET(cpu, ins->op1, CPU_CPUREG_GET(cpu, ins->op2));
}

static void cpu_op_movs(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t op2 = CPU_CPUREG_GETU(cpu, ins->op2);;

	CPU_CPUREG_SETU(cpu, ins->op1, op2);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !op2);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, op2 & CPU_SIGN_BIT);
}

static void cpu_op_add(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETU(cpu, ins->op1, CPU_CPUREG_GETU(cpu, ins->op2) + CPU_CPUREG_GETU(cpu, ins->op3));
}

static void cpu_op_adds(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 + op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_C, res < op2 || res < op3);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_V, (op2 & CPU_SIGN_BIT) == (op3 & CPU_SIGN_BIT) &&
							(res & CPU_SIGN_BIT) != (op2 & CPU_SIGN_BIT));
}

static void cpu_op_adc(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 + op3 + (CPU_STATUS_GETBIT(cpu, CPU_STATUS_C) ? 1 : 0);
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_C, res < op2 || res < op3);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_V, (op2 & CPU_SIGN_BIT) == (op3 & CPU_SIGN_BIT) &&
							(res & CPU_SIGN_BIT) != (op2 & CPU_SIGN_BIT));
}

static void cpu_op_sub(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETU(cpu, ins->op1, CPU_CPUREG_GETU(cpu, ins->op2) - CPU_CPUREG_GETU(cpu, ins->op3));
}


static void cpu_op_subs(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 - op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_C, !(res > op2));  /* borrow = !carry */
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_V, (op2 & CPU_SIGN_BIT) != (op3 & CPU_SIGN_BIT) &&
							(res & CPU_SIGN_BIT) == (op2 & CPU_SIGN_BIT));
}


static void cpu_op_sbc(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 - op3 + (CPU_STATUS_GETBIT(cpu, CPU_STATUS_C) ? 0 : -1);
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_C, !(res > op2));	/* borrow = !carry */
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_V, (op2 & CPU_SIGN_BIT) != (op3 & CPU_SIGN_BIT) &&
							(res & CPU_SIGN_BIT) == (op2 & CPU_SIGN_BIT));
}

static void cpu_op_mul(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETU(cpu, ins->op1, CPU_CPUREG_GETU(cpu, ins->op2) * CPU_CPUREG_GETU(cpu, ins->op3));
}

static void cpu_op_mls(rycpu_t *cpu, asmins_t *ins)
{
	r_int_t res;
	r_int_t op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = (r_uint_t)(op2 * op3);
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	/* TBD: Not sure how to update the CPU_STATUS_C */
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_muls(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 * op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_C, op2 && (res / op2) != op3);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_div(rycpu_t *cpu, asmins_t *ins)
{
	if (!CPU_CPUREG_GETU(cpu, ins->op3))
		CPU_ABORT(cpu, CPU_E_DIVZERO);

	CPU_CPUREG_SETU(cpu, ins->op1, CPU_CPUREG_GETU(cpu, ins->op2) / CPU_CPUREG_GETU(cpu, ins->op3));
}

static void cpu_op_dvs(rycpu_t *cpu, asmins_t *ins)
{
	r_int_t res;
	r_int_t op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	if (!op3)
		CPU_ABORT(cpu, CPU_E_DIVZERO);
	res = (r_uint_t)(op2 / op3);
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_divs(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res;
	r_uint_t op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	if (!op3)
		CPU_ABORT(cpu, CPU_E_DIVZERO);
	res = op2 / op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_mod(rycpu_t *cpu, asmins_t *ins)
{
	if (!CPU_CPUREG_GETU(cpu, ins->op3))
		CPU_ABORT(cpu, CPU_E_DIVZERO);

	CPU_CPUREG_SETU(cpu, ins->op1, CPU_CPUREG_GETU(cpu, ins->op2) % CPU_CPUREG_GETU(cpu, ins->op3));
}

static void cpu_op_mods(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	if (!op3)
		CPU_ABORT(cpu, CPU_E_DIVZERO);
	res = op2 % op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_asr(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 >> op3;
	res |= op2 & CPU_SIGN_BIT;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_swp(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t tmp = CPU_CPUREG_GET(cpu, ins->op1);
	CPU_CPUREG_SET(cpu, ins->op1, CPU_CPUREG_GET(cpu, ins->op2));
	CPU_CPUREG_SET(cpu, ins->op2, tmp);
}

static void cpu_op_inc(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETU(cpu, ins->op1, CPU_CPUREG_GETU(cpu, ins->op1) + 1);
}

static void cpu_op_dec(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETU(cpu, ins->op1, CPU_CPUREG_GETU(cpu, ins->op1) - 1);
}

static void cpu_op_lsl(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 << op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_lsr(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 >> op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
}


static void cpu_op_lsrs(rycpu_t *cpu, asmins_t *ins)
{
	r_int_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = ((r_uint_t)op2) >> op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
}

static void cpu_op_not(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2);

	res = ~op2;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_tst(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 & op3;
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);

}

static void cpu_op_teq(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 ^ op3;
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_and(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 & op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_orr(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 | op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_bic(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 & ~op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_clz(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2);

	for (res = CPU_REGISTER_BITS; op2; ) {
		op2 >>= 1;
		res -= 1;
	}
	CPU_CPUREG_SETU(cpu, ins->op1, res);
}

static void cpu_op_xor(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2 ^ op3;
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
}

static void cpu_op_bx(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETIP(cpu, PC, CPU_CPUREG_GETIP(cpu, ins->op1) - 1);
}

static void cpu_op_bxeq(rycpu_t *cpu, asmins_t *ins)
{
	if ((cpu->status & CPU_STATUS_Z)) {
		CPU_CPUREG_SETIP(cpu, PC, CPU_CPUREG_GETIP(cpu, ins->op1) - 1);
	}
}

static void cpu_op_bxneq(rycpu_t *cpu, asmins_t *ins)
{
	if (!(cpu->status & CPU_STATUS_Z)) {
		CPU_CPUREG_SETIP(cpu, PC, CPU_CPUREG_GETIP(cpu, ins->op1) - 1);
	}
}

static void cpu_op_bxleq(rycpu_t *cpu, asmins_t *ins)
{
	if ((cpu->status & CPU_STATUS_N) || (cpu->status & CPU_STATUS_Z)) {
		CPU_CPUREG_SETIP(cpu, PC, CPU_CPUREG_GETIP(cpu, ins->op1) - 1);
	}
}

static void cpu_op_bxgeq(rycpu_t *cpu, asmins_t *ins)
{
	if ((cpu->status & CPU_STATUS_N) == 0 || (cpu->status & CPU_STATUS_Z) == 1){
		CPU_CPUREG_SETIP(cpu, PC, CPU_CPUREG_GETIP(cpu, ins->op1) - 1);
	}
}

static void cpu_op_bxles(rycpu_t *cpu, asmins_t *ins)
{
	if ((cpu->status & CPU_STATUS_N)) {
		CPU_CPUREG_SETIP(cpu, PC, CPU_CPUREG_GETIP(cpu, ins->op1) - 1);
	}
}

static void cpu_op_bxgre(rycpu_t *cpu, asmins_t *ins)
{
	if ((cpu->status & CPU_STATUS_N) == 0 && (cpu->status & CPU_STATUS_Z) == 0) {
		CPU_CPUREG_SETIP(cpu, PC, CPU_CPUREG_GETIP(cpu, ins->op1) - 1);
	}
}

static void cpu_op_bxl(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETIP(cpu, LR, CPU_CPUREG_GETIP(cpu, PC));
	CPU_CPUREG_SETIP(cpu, PC, CPU_CPUREG_GETIP(cpu, ins->op1) - 1);
}

static void cpu_op_b(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t pc = 0;

	pc += CPU_CPUREG_GETU(cpu, ins->op1);
	CPU_CPUREG_INCIP(cpu, PC, pc - 1);
}

static void cpu_op_beq(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t pc = 0;

	if ((cpu->status & CPU_STATUS_Z)) {
		pc += CPU_CPUREG_GETU(cpu, ins->op1);
		CPU_CPUREG_INCIP(cpu, PC, pc - 1);
	}
}

static void cpu_op_bneq(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t pc = 0;

	if ((cpu->status & CPU_STATUS_Z) == 0) {
		pc += CPU_CPUREG_GETU(cpu, ins->op1);
		CPU_CPUREG_INCIP(cpu, PC, pc - 1);
	}
}

static void cpu_op_bleq(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t pc = 0;

	if ((cpu->status & CPU_STATUS_N) || (cpu->status & CPU_STATUS_Z)) {
//		if (ins->op1 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op1);
//		if (ins->op2 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op2);
//		if (ins->op3 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op3);
		pc += CPU_CPUREG_GETU(cpu, ins->op1);
		CPU_CPUREG_INCIP(cpu, PC, pc - 1);
	}
}

static void cpu_op_bgeq(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t pc = 0;

	if ((cpu->status & CPU_STATUS_N) == 0 || (cpu->status & CPU_STATUS_Z) == 1) {
//		if (ins->op1 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op1);
//		if (ins->op2 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op2);
//		if (ins->op3 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op3);
		pc += CPU_CPUREG_GETU(cpu, ins->op1);
		CPU_CPUREG_INCIP(cpu, PC, pc - 1);
	}
}


static void cpu_op_bles(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t pc = 0;


	if ((cpu->status & CPU_STATUS_N)) {
//		if (ins->op1 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op1);
//		if (ins->op2 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op2);
//		if (ins->op3 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op3);
		pc += CPU_CPUREG_GETU(cpu, ins->op1);
		CPU_CPUREG_INCIP(cpu, PC, pc - 1);
	}
}


static void cpu_op_bgre(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t pc = 0;

	if ((cpu->status & CPU_STATUS_N) == 0 && (cpu->status & CPU_STATUS_Z) == 0) {
//		if (ins->op1 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op1);
//		if (ins->op2 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op2);
//		if (ins->op3 != XX)
//			pc += CPU_CPUREG_GETU(cpu, ins->op3);
		pc += CPU_CPUREG_GETU(cpu, ins->op1);
		CPU_CPUREG_INCIP(cpu, PC, pc - 1);
	}
}

static void cpu_op_bl(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t pc = 0;

	pc += CPU_CPUREG_GETU(cpu, ins->op1);
	CPU_CPUREG_SETIP(cpu, LR, CPU_CPUREG_GETIP(cpu, PC));
	CPU_CPUREG_INCIP(cpu, PC, pc - 1);
}

static void cpu_op_ret(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETU(cpu, PC, CPU_CPUREG_GETU(cpu, LR));
}

static void cpu_op_str(rycpu_t *cpu, asmins_t *ins)
{
	*((r_uint_t*)CPU_CPUREG_GETP(cpu, ins->op2)) = CPU_CPUREG_GETU(cpu, ins->op1);
}

static void cpu_op_strp(rycpu_t *cpu, asmins_t *ins)
{
	*((r_pointer_t*)CPU_CPUREG_GETP(cpu, ins->op2)) = (r_pointer_t)CPU_CPUREG_GETP(cpu, ins->op1);
}

static void cpu_op_strb(rycpu_t *cpu, asmins_t *ins)
{
	*((uint8_t*)CPU_CPUREG_GETP(cpu, ins->op2)) = (uint8_t)CPU_CPUREG_GETU(cpu, ins->op1);
}

static void cpu_op_strh(rycpu_t *cpu, asmins_t *ins)
{
	*((uint16_t*)CPU_CPUREG_GETP(cpu, ins->op2)) = (uint16_t)CPU_CPUREG_GETU(cpu, ins->op1);

}

static void cpu_op_strw(rycpu_t *cpu, asmins_t *ins)
{
	*((uint32_t*)CPU_CPUREG_GETP(cpu, ins->op2)) = (uint32_t)CPU_CPUREG_GETU(cpu, ins->op1);

}

static void cpu_op_strr(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t *dest = (varreg_t *)CPU_CPUREG_GETP(cpu, ins->op2);
	*dest = CPU_CPUREG_GET(cpu, ins->op1);
}

static void cpu_op_ldr(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETU(cpu, ins->op1, *((r_uint_t*)CPU_CPUREG_GETU(cpu, ins->op2)));

}

static void cpu_op_ldrp(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETP(cpu, ins->op1, CPU_CPUREG_GETP(cpu, ins->op2));
}


static void cpu_op_ldrb(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETU(cpu, ins->op1, *((uint8_t*)CPU_CPUREG_GETU(cpu, ins->op2)));
}


static void cpu_op_ldrh(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETU(cpu, ins->op1, *((uint16_t*)CPU_CPUREG_GETU(cpu, ins->op2)));
}

static void cpu_op_ldrw(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SETU(cpu, ins->op1, *((uint32_t*)CPU_CPUREG_GETU(cpu, ins->op2)));
}

static void cpu_op_ldrr(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_SET(cpu, ins->op1, *((varreg_t*)CPU_CPUREG_GETU(cpu, ins->op2)));
}

static void cpu_op_cflag(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t op1 = CPU_CPUREG_GETU(cpu, ins->op1);
	CPU_STATUS_CLRBIT(cpu, op1);
}

static void cpu_op_clr(rycpu_t *cpu, asmins_t *ins)
{
	CPU_CPUREG_CLEAR(cpu, ins->op1);
}

static void cpu_op_clrr(rycpu_t *cpu, asmins_t *ins)
{
	CPU_REG_CLEAR(*((varreg_t*)CPU_CPUREG_GETP(cpu, ins->op1)));
}

static void cpu_op_stm(rycpu_t *cpu, asmins_t *ins)
{
	int n;
	r_uint_t *dst = (r_uint_t*)	CPU_CPUREG_GETU(cpu, ins->op1);
	r_uint_t bits = CPU_CPUREG_GETU(cpu, ins->op2);

	for (n = 0; bits && n < RLST; n++) {
		if (((r_uint_t)(1 << n)) & bits) {
			*dst = CPU_CPUREG_GETU(cpu, n);
			dst += 1;
			bits &= ~(1<<n);
		}
	}
}

static void cpu_op_ldm(rycpu_t *cpu, asmins_t *ins)
{
	int n;
	r_uint_t *src = (r_uint_t*)CPU_CPUREG_GETU(cpu, ins->op1);
	r_uint_t bits = CPU_CPUREG_GETU(cpu, ins->op2);

	for (n = 0; bits && n < RLST; n++) {
		if (((r_uint_t)(1 << n)) & bits) {
			CPU_CPUREG_CLEAR(cpu, n);
			CPU_CPUREG_SETU(cpu, n, *src);
			src += 1;
			bits &= ~(1<<n);
		}
	}
}

static void cpu_op_sts(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t sp = ((ins->op2 != XX) ? CPU_CPUREG_GETU(cpu, ins->op2) : 0) + ((ins->op3 != XX) ? CPU_CPUREG_GETU(cpu, ins->op3) : 0);

	if (sp >= cpu->stack.size())
		// cpu->stack.resize(sp + 1);
	CPU_STACK_WRITE(cpu, sp, CPU_CPUREG_GET(cpu, ins->op1));
}

static void cpu_op_lds(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t sp = ((ins->op2 != XX) ? CPU_CPUREG_GETU(cpu, ins->op2) : 0) + ((ins->op3 != XX) ? CPU_CPUREG_GETU(cpu, ins->op3) : 0);

	CPU_CPUREG_SET(cpu, ins->op1, CPU_STACK_READ(cpu, sp));
}

static void cpu_op_push(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP) + 1;
	// cpu->stack.resize(sp + 1);
	CPU_STACK_CHECKSIZE(cpu, sp);
	CPU_STACK_WRITE(cpu, sp, CPU_CPUREG_GET(cpu, ins->op1));
	CPU_CPUREG_SETU(cpu, SP, sp);
}

static void cpu_op_pop(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP);
	if (sp == 0)
		CPU_ABORT(cpu, CPU_E_EMPTY);
	CPU_CPUREG_SET(cpu, ins->op1, CPU_STACK_MOVE(cpu, sp));
	if (ins->op1 != SP)
		CPU_CPUREG_SETU(cpu, SP, sp - 1);
	// cpu->stack.resize(sp);
}

static void cpu_op_pushm(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t n, i = 0;
	r_uint_t bits = CPU_CPUREG_GETU(cpu, ins->op1);
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP);

	if (!(bits & ((1<<(CPU_REGS_NUM / 2)) - 1)))
		i = CPU_REGS_NUM / 2;
	for (;bits && i < RLST; i++) {
		n = ((r_uint_t)1) << i;
		if (n & bits) {
			sp += 1;
			CPU_STACK_WRITE(cpu, sp, CPU_CPUREG_GET(cpu, i));
			bits &= ~n;
		}
	}
	CPU_CPUREG_SETU(cpu, SP, sp);
}

static void cpu_op_popm(rycpu_t *cpu, asmins_t *ins)
{
	int n, i = RLST - 1;
	r_uint_t bits = CPU_CPUREG_GETU(cpu, ins->op1);
	r_uint_t savedbits = bits;
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP);

	if (!(bits & ~((1 << (CPU_REGS_NUM / 2)) - 1)))
		i = CPU_REGS_NUM / 2 - 1;
	for (; bits && i >= 0; i--) {
		n = 1 << i;
		if (n & bits) {
			CPU_CPUREG_SET(cpu, i, CPU_STACK_MOVE(cpu, sp));
			sp -= 1;
			bits &= ~n;
		}
	}
	if (!(((r_uint_t)(1 << SP)) & savedbits))
		CPU_CPUREG_SETU(cpu, SP, sp);
}

static void cpu_op_cmp(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t op1 = CPU_CPUREG_GETU(cpu, ins->op1), op2 = CPU_CPUREG_GETU(cpu, ins->op2);
	r_uint_t res = (r_uint_t)(op1 - op2);

	CPU_STATUS_UPDATE(cpu, CPU_STATUS_C, op1 < op2);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_V, (op1 & CPU_SIGN_BIT) != (op2 & CPU_SIGN_BIT) &&
							(res & CPU_SIGN_BIT) == (op1 & CPU_SIGN_BIT));
}

static void cpu_op_cmn(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t op1 = CPU_CPUREG_GETU(cpu, ins->op1), op2 = CPU_CPUREG_GETU(cpu, ins->op2);
	r_uint_t res = (r_uint_t)(op1 + op2);

	CPU_STATUS_UPDATE(cpu, CPU_STATUS_C, res < op1 || res < op2);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_V,
		(op1 & CPU_SIGN_BIT) == (op2 & CPU_SIGN_BIT) && (res & CPU_SIGN_BIT) != (op1 & CPU_SIGN_BIT));
}

static void cpu_op_nop(rycpu_t *cpu, asmins_t *ins)
{
}

static void cpu_op_ror(rycpu_t *cpu, asmins_t *ins)
{
	unsigned int i;
	r_uint_t res, op2 = CPU_CPUREG_GETU(cpu, ins->op2), op3 = CPU_CPUREG_GETU(cpu, ins->op3);

	res = op2;
	for (i = 0; i < op3; i++) {
		if (res & 1) {
			res >>= 1;
			res |= CPU_SIGN_BIT;
		} else {
			res >>= 1;
		}
	}
	CPU_CPUREG_SETU(cpu, ins->op1, res);
	CPU_STATUS_CLRALL(cpu);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, !res);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_N, res & CPU_SIGN_BIT);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_C, res & CPU_SIGN_BIT);
}

static void cpu_op_addrs(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t sp = ((ins->op2 != XX) ? CPU_CPUREG_GETU(cpu, ins->op2) : 0) + ((ins->op3 != XX) ? CPU_CPUREG_GETU(cpu, ins->op3) : 0);

	CPU_STACK_CHECKSIZE(cpu, sp);
	CPU_CPUREG_CLEAR(cpu, ins->op1);
	CPU_CPUREG_SETP(cpu, ins->op1, CPU_STACK_ADDR(cpu, sp));
}

static void cpu_ryo_stack_emplace(rycpu_t *cpu, asmins_t *ins, RyObject* obj)
{
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP) + 1;
	CPU_STACK_CHECKSIZE(cpu, sp);
	CPU_STACK_EMPLACE(cpu, sp, obj);
	CPU_CPUREG_SETU(cpu, SP, sp);
}

static void cpu_ryo_clone(rycpu_t *cpu, asmins_t *ins)
{
	const varreg_t& reg2 = CPU_CPUREG_GET(cpu, ins->op2);

	if (!std::holds_alternative<RyPointer>(reg2))
		throw InvalidTypeError("Unsupported type");
	RyObject* o2 = std::get<RyPointer>(reg2).get();
	RyObject* o1 = o2->Clone();
	CPU_CPUREG_EMPLACE(cpu, ins->op1, o1);
}

static void cpu_ryo_call(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t& op1 = CPU_CPUREG_GET(cpu, ins->op1);
	RyObject* o1 = std::get<RyPointer>(op1).get();
	if (CheckType_Function(o1)) {
		CodeFragment& code = static_cast<FunctionObject*>(o1)->value_;
		asmins_t *prog = code.CurrentBlock().data();
		CPU_CPUREG_EMPLACE(cpu, ins->op1, new IntObject(0));
		CPU_CPUREG_EMPLACE(cpu, CO, &code.objects_, true);
		CPU_CPUREG_SETIP(cpu, LR, CPU_CPUREG_GETIP(cpu, PC));
		CPU_CPUREG_SETIP(cpu, PC, prog - 1);
	} else if (CheckType_Int(o1)) {
		rycpu_swi_t swi = (rycpu_swi_t) RyObject_GetInt(o1);
		(*swi)(cpu, ins);
	} else {
		throw InvalidTypeError(std::string("The argument must be of type FunctionObject or SWI, not ") + o1->GetType());

	}
}


static void cpu_ryo_ptostr(rycpu_t *cpu, asmins_t *ins)
{
	const varreg_t& reg = CPU_CPUREG_GET(cpu, ins->op2);

	if (std::holds_alternative<RyPointer>(reg))
		throw InvalidTypeError("Unsupported type");
	RyObject* obj = ((RyObject*)new StringObject((const char*)CPU_REG_GETP(reg)));
	CPU_CPUREG_EMPLACE(cpu, ins->op1, obj);
}

static void cpu_ryo_ptoint(rycpu_t *cpu, asmins_t *ins)
{
	const varreg_t& reg = CPU_CPUREG_GET(cpu, ins->op2);

	if (std::holds_alternative<RyPointer>(reg))
		throw InvalidTypeError("Unsupported type");
	RyObject* obj = ((RyObject*)new IntObject(*((r_int_t*)CPU_REG_GETP(reg))));
	CPU_CPUREG_EMPLACE(cpu, ins->op1, obj);
}

static void cpu_ryo_utoint(rycpu_t *cpu, asmins_t *ins)
{
	const varreg_t& reg = CPU_CPUREG_GET(cpu, ins->op2);

	if (std::holds_alternative<RyPointer>(reg))
		throw InvalidTypeError("Unsupported type");
	CPU_CPUREG_EMPLACE(cpu, ins->op1, RyPointer(new IntObject(std::get<r_uint_t>(reg))));
}

static void cpu_ryo_movref(rycpu_t *cpu, asmins_t *ins)
{
	const varreg_t& reg = CPU_CPUREG_GET(cpu, ins->op2);

	if (!std::holds_alternative<RyPointer>(reg))
		throw InvalidTypeError("Unsupported type");
	CPU_CPUREG_EMPLACE(cpu, ins->op1, std::get<RyPointer>(reg).get(), true);
}

static void cpu_ryo_unref(rycpu_t *cpu, asmins_t *ins)
{
	const varreg_t& reg = CPU_CPUREG_GET(cpu, ins->op1);

	if (std::holds_alternative<RyPointer>(reg) && std::get<RyPointer>(reg).ref)
		CPU_CPUREG_EMPLACE(cpu, ins->op1, std::get<RyPointer>(reg).get()->Clone());
}


static void cpu_ryo_ptoref(rycpu_t *cpu, asmins_t *ins)
{
	const varreg_t& reg = CPU_CPUREG_GET(cpu, ins->op2);

	if (!std::holds_alternative<r_uint_t>(reg))
		throw InvalidTypeError("Unsupported type");
	CPU_CPUREG_EMPLACE(cpu, ins->op1, (RyObject*)std::get<r_uint_t>(reg), true);
}

static void cpu_ryo_at(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t& op2 = CPU_CPUREG_GET(cpu, ins->op2);
	varreg_t& op3 = CPU_CPUREG_GET(cpu, ins->op3);
	RyObject* o1;
	RyObject* o2 = std::get<RyPointer>(op2).get();
	if (std::holds_alternative<r_uint_t>(op3)) {
		o1 = o2->At(std::get<r_uint_t>(op3));
	} else {
		o1 = o2->At(std::get<RyPointer>(op3).get());
	}
	CPU_CPUREG_EMPLACE(cpu, ins->op1, o1, true);
}

static void cpu_ryo_assignat(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t& op1 = CPU_CPUREG_GET(cpu, ins->op1);
	varreg_t& op2 = CPU_CPUREG_GET(cpu, ins->op2);
	varreg_t& op3 = CPU_CPUREG_GET(cpu, ins->op3);
	RyObject* o1 = std::get<RyPointer>(op1).get();
	RyObject* o2 = std::get<RyPointer>(op2).get();
	if (std::holds_alternative<r_uint_t>(op3)) {
		o1 = o2->AssignAt(std::get<r_uint_t>(op3), o1);
	} else {
		o1 = o2->AssignAt(std::get<RyPointer>(op3).get(), o1);
	}
}

static void cpu_ryo_assign(rycpu_t *cpu, asmins_t *ins)
{
	/*
	 * OP1 holds the variable map
	 * OP2 holds the number of parameters on the stack
	 */
	r_uint_t n = CPU_CPUREG_GETU(cpu, ins->op2);
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP);
	RyObject* pvars = std::get<RyPointer>(CPU_CPUREG_GET(cpu, ins->op1)).get();
	RyObject* pid = std::get<RyPointer>(CPU_STACK_READ(cpu, sp + 1 - n)).get();
	for (r_uint_t i = 1; i < (n - 1); i++) {
		pvars = pvars->At(pid);
		pid = std::get<RyPointer>(CPU_STACK_READ(cpu, sp + 1 - n + i)).get();
	}
	varreg_t& assignee = CPU_STACK_READ(cpu, sp);
	RyObject* passignee = std::get<RyPointer>(assignee).get();
	passignee = pvars->AssignAt(pid, passignee);
	CPU_STACK_WRITE(cpu, sp - n + 1, assignee);
	CPU_CPUREG_SETU(cpu, SP, sp - n + 1);
}

static void cpu_ryo_ref(rycpu_t *cpu, asmins_t *ins)
{
	/*
	 * OP1 holds the variable map
	 * OP2 holds the number of parameters on the stack
	 */
	r_uint_t n = CPU_CPUREG_GETU(cpu, ins->op2);
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP);
	RyObject* pvars = std::get<RyPointer>(CPU_CPUREG_GET(cpu, ins->op1)).get();
	RyObject* pid = std::get<RyPointer>(CPU_STACK_READ(cpu, sp + 1 - n)).get();
	for (r_uint_t i = 1; i < n; i++) {
		pvars = pvars->At(pid);
		pid = std::get<RyPointer>(CPU_STACK_READ(cpu, sp + 1 - n + i)).get();
	}
	CPU_STACK_ADDR(cpu, sp - n + 1)->emplace<RyPointer>(pvars->At(pid), true);
	CPU_CPUREG_SETU(cpu, SP, sp - n + 1);
}

static void cpu_ryo_refext(rycpu_t *cpu, asmins_t *ins)
{
	/*
	 * Extended lookup in two maps
	 * OP1 holds the variable map 1
	 * OP2 holds the variable map 2
	 * OP3 holds the number of parameters on the stack
	 */
	r_uint_t n = CPU_CPUREG_GETU(cpu, ins->op3);
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP);
	RyObject* pvars2 = std::get<RyPointer>(CPU_CPUREG_GET(cpu, ins->op2)).get();
	RyObject* pvars = std::get<RyPointer>(CPU_CPUREG_GET(cpu, ins->op1)).get();
	RyObject* pid = std::get<RyPointer>(CPU_STACK_READ(cpu, sp + 1 - n)).get();
	RyMap& map = RyObject_GetMap(pvars);
	RyPointer keyref(pid, true);
	RyMap::const_iterator it = map.find(keyref);
	if (it == map.end()) {
		pvars = pvars2;
	}
	for (r_uint_t i = 1; i < n; i++) {
		pvars = pvars->At(pid);
		pid = std::get<RyPointer>(CPU_STACK_READ(cpu, sp + 1 - n + i)).get();
	}
	CPU_STACK_ADDR(cpu, sp - n + 1)->emplace<RyPointer>(pvars->At(pid), true);
	CPU_CPUREG_SETU(cpu, SP, sp - n + 1);
}


static void cpu_ryo_append(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t& op1 = CPU_CPUREG_GET(cpu, ins->op1);
	varreg_t& op2 = CPU_CPUREG_GET(cpu, ins->op2);
	RyObject* o1 = std::get<RyPointer>(op1).get();
	RyObject* o2 = std::get<RyPointer>(op2).get();
	o2->Append(o1->Clone());
}

static void cpu_ryo_functop(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t& op2 = CPU_CPUREG_GET(cpu, ins->op2);
	RyObject* o2 = std::get<RyPointer>(op2).get();
	if (!CheckType_Function(o2))
		throw InvalidTypeError(std::string("The argument must be of type FunctionObject, not ") + o2->GetType());
	asmins_t *prog = static_cast<FunctionObject*>(o2)->value_.CurrentBlock().data();
	CPU_CPUREG_SETP(cpu, ins->op1, prog);
}

typedef RyObject* (RyObject::*binop)(RyObject*, RyObject*) const;

static void cpu_obj_binop(rycpu_t *cpu, asmins_t *ins, binop func)
{
	varreg_t& op2 = CPU_CPUREG_GET(cpu, ins->op2);
	varreg_t& op3 = CPU_CPUREG_GET(cpu, ins->op3);
	RyObject* o2 = std::get<RyPointer>(op2).get();
	RyObject* o3 = std::get<RyPointer>(op3).get();
	RyObject* o1 = (o2->*func)(o2, o3);
	if (o1 == nullptr)
		o1 = (o3->*func)(o2, o3);
	CPU_CPUREG_EMPLACE(cpu, ins->op1, o1);
}

static void cpu_ryo_neg(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t& op1 = CPU_CPUREG_GET(cpu, ins->op1);
	RyObject* o1 = std::get<RyPointer>(op1).get();
	CPU_CPUREG_EMPLACE(cpu, ins->op1, o1->Negate());
}

static void cpu_ryo_inc(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t& op1 = CPU_CPUREG_GET(cpu, ins->op1);
	RyObject* o1 = std::get<RyPointer>(op1).get();
	o1->Inc();
}

static void cpu_ryo_dec(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t& op1 = CPU_CPUREG_GET(cpu, ins->op1);
	RyObject* o1 = std::get<RyPointer>(op1).get();
	o1->Dec();
}

static void cpu_ryo_boolean(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t sp = CPU_CPUREG_GETU(cpu, SP);
	RyObject* o = std::get<RyPointer>(CPU_STACK_READ(cpu, sp)).get();
	RyObject* b = o->Boolean();
	CPU_STACK_EMPLACE(cpu, sp, b);
	CPU_STATUS_UPDATE(cpu, CPU_STATUS_Z, RyObject_GetBool(b));
}

static void cpu_ryo_array(rycpu_t *cpu, asmins_t *ins)
{
	cpu_ryo_stack_emplace(cpu, ins, new ArrayObject());
}

static void cpu_ryo_map(rycpu_t *cpu, asmins_t *ins)
{
	cpu_ryo_stack_emplace(cpu, ins, new MapObject());
}

static void cpu_ryo_add(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::Add);
}

static void cpu_ryo_sub(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::Subtract);
}

static void cpu_ryo_mul(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::Multiply);
}

static void cpu_ryo_div(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::Divide);
}

static void cpu_ryo_mod(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::Modulus);
}

static void cpu_ryo_lsl(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::LeftShift);
}

static void cpu_ryo_lsr(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::RightShift);
}

static void cpu_ryo_eq(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::Equal);
}

static void cpu_ryo_ne(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::NotEqual);
}

static void cpu_ryo_lt(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::Less);
}

static void cpu_ryo_gt(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::Greater);
}

static void cpu_ryo_le(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::LessEqual);
}

static void cpu_ryo_ge(rycpu_t *cpu, asmins_t *ins)
{
	cpu_obj_binop(cpu, ins, &RyObject::GreaterEqual);
}

static void cpu_ryo_ldsref(rycpu_t *cpu, asmins_t *ins)
{
	r_uint_t sp = ((ins->op2 != XX) ? CPU_CPUREG_GETU(cpu, ins->op2) : 0) + ((ins->op3 != XX) ? CPU_CPUREG_GETU(cpu, ins->op3) : 0);
	if (!std::holds_alternative<RyPointer>(CPU_STACK_READ(cpu, sp)))
		throw InvalidTypeError("Invalid stack operation");
	RyObject* o1 = std::get<RyPointer>(CPU_STACK_READ(cpu, sp)).get();
	CPU_CPUREG_EMPLACE(cpu, ins->op1, o1, true);
}


#if 0
static void obj_op_arrld(rycpu_t *cpu, asmins_t *ins)
{
	ArrayObject* data = (ArrayObject*)CPU_CPUREG_GETP(cpu, ins->op2);
	r_uint_t idx = CPU_CPUREG_GETU(cpu, ins->op3);
	RyObject* obj = data->value_[idx].get();
	CPU_CPUREG_SETP(cpu, ins->op1, obj);
}

static void obj_op_add(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t b = ry_cpu_pop(cpu, NULL);
	varreg_t a = ry_cpu_pop(cpu, NULL);
	varreg_t c;
	RyObject *o1 = (RyObject*)a.p;
	RyObject *o2 = (RyObject*)b.p;
	c.p = o1->Add(o1, o2);
	if (c.p == nullptr)
		c.p = o2->Add(o1, o2);
	ry_cpu_push(cpu, c, NULL);
}

static void obj_op_mul(rycpu_t *cpu, asmins_t *ins)
{
	varreg_t b = ry_cpu_pop(cpu, NULL);
	varreg_t a = ry_cpu_pop(cpu, NULL);
	varreg_t c;
	RyObject *o1 = (RyObject*)a.p;
	RyObject *o2 = (RyObject*)b.p;
	c.p = o1->Multiply(o1, o2);
	if (c.p == nullptr)
		c.p = o2->Multiply(o1, o2);
	ry_cpu_push(cpu, c, NULL);
}
#endif

}

