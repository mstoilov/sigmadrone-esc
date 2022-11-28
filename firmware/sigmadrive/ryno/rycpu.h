#ifndef _RYCPU_H_
#define _RYCPU_H_

#include <cstdint>
#include <variant>
#include <cassert>
#include "ryobject.h"

namespace ryno {

#ifndef RMV_UINT
#define RMV_UINT size_t
#endif

#ifndef RMV_INT
#define RMV_INT ssize_t
#endif

typedef RMV_UINT r_uint_t;
typedef RMV_INT r_int_t;
typedef void* r_pointer_t;

#define CPU_REGISTER_BITS (8 * sizeof(void*))
#define CPU_SIGN_BIT (((r_uint_t)1) << (CPU_REGISTER_BITS - 1))
#define CPU_STATUS_Z (1 << 0)
#define CPU_STATUS_N (1 << 1)
#define CPU_STATUS_C (1 << 2)
#define CPU_STATUS_V (1 << 3)
#define CPU_STATUS_E (1 << 4)
#define CPU_STATUS_ALL (CPU_STATUS_Z | CPU_STATUS_N | CPU_STATUS_C | CPU_STATUS_V | CPU_STATUS_E)
#define CPU_STATUS_GETBIT(cpu, bit) ((cpu)->status & (bit))
#define CPU_STATUS_SETBIT(cpu, bit) do { (cpu)->status |= (bit); } while (0)
#define CPU_STATUS_CLRBIT(cpu, bit) do { (cpu)->status &= ~(bit); } while (0)
#define CPU_STATUS_CLRALL(cpu) CPU_STATUS_CLRBIT(cpu, CPU_STATUS_ALL)
#define CPU_STATUS_UPDATE(cpu, b, c) \
do { \
    if (c) \
        CPU_STATUS_SETBIT(cpu, b); \
    else \
        CPU_STATUS_CLRBIT(cpu, b); \
} while (0)


#define R0 0
#define R1 1
#define R2 2
#define R3 3
#define R4 4
#define R5 5
#define R6 6
#define R7 7
#define R8 8
#define R9 9
#define R10 10
#define R11 11
#define R12 12
#define R13 13
#define R14 14
#define R15 15
#define RLST R15
#define LP (RLST - 8)
#define GP (RLST - 7)
#define CO (RLST - 6)
#define FP (RLST - 5)
#define SP (RLST - 4)
#define LR (RLST - 3)
#define PC (RLST - 2)
#define DA (RLST - 1)				/* The DA register should never be modified manually, otherwise the result is undefined */
#define XX (RLST)
#define CPU_REGS_NUM (RLST + 1)

#define R_ASSERT(__a__) do { assert(__a__); } while (0)
#define R_SIZE_ALIGN(s, n) ((((s) + (n) - 1) / (n)) * (n))
#define R_MIN(a, b) ((a) < (b) ? (a): (b))
#define R_MAX(a, b) ((a) > (b) ? (a): (b))

#define CPU_ABORT(__cpu__, __e__) do { __cpu__->error = (__e__); (__cpu__)->abort = 1; R_ASSERT(0); return; } while (0)
#define BIT(__shiftby__) (1 << (__shiftby__))
#define BITS(__f__, __l__)  (BITR(__f__, __l__, R0)) | (BITR(__f__, __l__, R1)) | (BITR(__f__, __l__, R2)) | (BITR(__f__, __l__, R3)) | \
							(BITR(__f__, __l__, R4)) | (BITR(__f__, __l__, R5)) | (BITR(__f__, __l__, R6)) | (BITR(__f__, __l__, R7)) | \
							(BITR(__f__, __l__, R8)) | (BITR(__f__, __l__, R9)) | (BITR(__f__, __l__, R10)) | (BITR(__f__, __l__, R11)) | \
							(BITR(__f__, __l__, R12)) | (BITR(__f__, __l__, R13)) | (BITR(__f__, __l__, R14)) | (BITR(__f__, __l__, R15))


#define CPU_OPCODE_BITS 8
#define CPU_SWI_TABLE_BITS 8
#define CPU_SWI_NUM_BITS 8
#define CPU_SWI_TABLE(__op__) ((__op__) >> CPU_SWI_NUM_BITS)
#define CPU_SWI_NUM(__op__) ((__op__) & ((1 << CPU_SWI_NUM_BITS) - 1))
#define CPU_SWI_ID(__t__, __o__) ((((__t__) & ((1 << CPU_SWI_TABLE_BITS) - 1))  << CPU_SWI_NUM_BITS) | ((__o__) & ((1 << CPU_SWI_NUM_BITS) - 1)))
#define CPU_ASMINS_OPCODE(__opcode__) ((__opcode__) & ((1 << CPU_OPCODE_BITS) - 1))
#define CPU_ASMINS_SWI(__opcode__) ((__opcode__) >> CPU_OPCODE_BITS)


#define CPU_E_NONE			(0)
#define CPU_E_DIVZERO		(1)
#define CPU_E_ILLEGAL		(2)
#define CPU_E_CAST			(3)
#define CPU_E_SWINUM		(4)
#define CPU_E_SWITABLE		(5)
#define CPU_E_LVALUE		(6)
#define CPU_E_NOMEM			(7)
#define CPU_E_EMPTY			(8)
#define CPU_E_NOTFUNCTION	(9)
#define CPU_E_USERABORT		(10)

using varreg_t = std::variant<r_uint_t, RyPointer>;

struct swi_table_t;

struct rycpu_t {
	varreg_t r[16];
	r_uint_t status;
	r_uint_t error;
	r_uint_t abort;
	std::vector<varreg_t> stack;
	swi_table_t *switables;
	void *userdata0;
	void *userdata1;
	void *userdata2;
	void *userdata3;
};


struct asmins_t {
	r_uint_t data;
	uint16_t opcode:8;
	uint16_t cond:8;
	uint16_t op1:4;
	uint16_t op2:4;
	uint16_t op3:4;
	uint16_t da:1;
	uint16_t re:1;
};

inline varreg_t CPU_CPUREG_GET(const rycpu_t* __cpu__, size_t __r__) { return __cpu__->r[__r__]; }
inline varreg_t& CPU_CPUREG_GET(rycpu_t* __cpu__, size_t __r__) { return __cpu__->r[__r__]; }
inline void CPU_CPUREG_SET(rycpu_t* __cpu__, size_t __r__, const varreg_t& val) { __cpu__->r[__r__] = val; }
inline void CPU_CPUREG_SET(rycpu_t* __cpu__, size_t __r__, varreg_t&& val) { __cpu__->r[__r__] = std::move(val); }

template<typename T, typename...ARGS>
inline void CPU_REG_EMPLACE(varreg_t& reg, const T& val, ARGS...args)
{
	if constexpr (std::is_same<std::remove_cv_t<T>, r_uint_t>::value) {
		reg.emplace<r_uint_t>(val, args...);
	} else {
		reg.emplace<RyPointer>(val, args...);
	}
}

template<typename T, typename...ARGS>
inline void CPU_CPUREG_EMPLACE(rycpu_t* __cpu__, size_t __r__, const T& val, ARGS...args)
{
	CPU_REG_EMPLACE(__cpu__->r[__r__], val, args...);
}


inline r_uint_t CPU_REG_GETU(const varreg_t& __reg__) { return std::get<r_uint_t>(__reg__); }
inline void CPU_REG_SETU(varreg_t& __reg__, r_uint_t __val__) { __reg__ = (r_uint_t)(__val__); }
inline r_uint_t CPU_CPUREG_GETU(const rycpu_t* __cpu__, size_t __r__) { return CPU_REG_GETU(__cpu__->r[__r__]); }
inline void CPU_CPUREG_SETU(rycpu_t* __cpu__, size_t __r__, r_uint_t __val__) {CPU_REG_SETU(__cpu__->r[__r__], __val__); }

inline r_pointer_t CPU_REG_GETP(const varreg_t& __reg__) { return (r_pointer_t)std::get<r_uint_t>(__reg__); }
inline void CPU_REG_SETP(varreg_t& __reg__, r_pointer_t __val__) { __reg__ = (r_uint_t)(__val__); }
inline r_pointer_t CPU_CPUREG_GETP(rycpu_t* __cpu__, size_t __r__) { return CPU_REG_GETP(__cpu__->r[__r__]); }
inline void CPU_CPUREG_SETP(rycpu_t* __cpu__, size_t __r__, r_pointer_t __val__) {CPU_REG_SETP(__cpu__->r[__r__], __val__); }

inline asmins_t* CPU_REG_GETIP(const varreg_t& __reg__) { return (asmins_t*)((void*)std::get<r_uint_t>(__reg__)); }
inline void CPU_REG_SETIP(varreg_t& __reg__, asmins_t* __val__) { __reg__ = (r_uint_t)((r_pointer_t)(__val__)); }
inline void CPU_REG_INCIP(varreg_t& __reg__, size_t __val__) {asmins_t *p = CPU_REG_GETIP(__reg__); CPU_REG_SETIP(__reg__, (p + (__val__))); }

inline asmins_t* CPU_CPUREG_GETIP(rycpu_t* __cpu__, size_t __r__) { return CPU_REG_GETIP(__cpu__->r[__r__]); }
inline void CPU_CPUREG_SETIP(rycpu_t* __cpu__, size_t __r__, asmins_t* __val__) {CPU_REG_SETIP(__cpu__->r[__r__], __val__); }
inline void CPU_CPUREG_INCIP(rycpu_t* __cpu__, size_t __r__, size_t __val__) {CPU_REG_INCIP(__cpu__->r[__r__], __val__); }

inline void CPU_REG_CLEAR(varreg_t& __reg__) { std::get<r_uint_t>((__reg__)) = 0UL; }
inline void CPU_CPUREG_CLEAR(rycpu_t* __cpu__, size_t __r__) { CPU_REG_CLEAR(__cpu__->r[__r__]); }

inline void CPU_STACK_CHECKSIZE(rycpu_t*  __cpu__, size_t __sp__) {	if (__sp__ >= __cpu__->stack.size()) __cpu__->stack.resize(__sp__ * 2 + 1); }
inline varreg_t* CPU_STACK_ADDR(rycpu_t*  __cpu__, size_t __sp__) { CPU_STACK_CHECKSIZE(__cpu__, __sp__); return &__cpu__->stack[__sp__]; }
inline varreg_t& CPU_STACK_READ(rycpu_t*  __cpu__, size_t __sp__) { CPU_STACK_CHECKSIZE(__cpu__, __sp__); return __cpu__->stack[__sp__]; }
inline varreg_t&& CPU_STACK_MOVE(rycpu_t*  __cpu__, size_t __sp__) { CPU_STACK_CHECKSIZE(__cpu__, __sp__); return std::move(__cpu__->stack[__sp__]); }
inline void CPU_STACK_WRITE(rycpu_t*  __cpu__, size_t __sp__, const varreg_t& val) { CPU_STACK_CHECKSIZE(__cpu__, __sp__); __cpu__->stack[__sp__] = val; }
inline void CPU_STACK_WRITE(rycpu_t*  __cpu__, size_t __sp__, varreg_t&& val) { CPU_STACK_CHECKSIZE(__cpu__, __sp__); __cpu__->stack[__sp__] = std::move(val); }
inline void CPU_STACK_CLEAR(rycpu_t*  __cpu__, size_t __sp__) { CPU_STACK_CHECKSIZE(__cpu__, __sp__); __cpu__->stack[__sp__] = varreg_t(); }

template<typename T, typename...ARGS>
inline void CPU_STACK_EMPLACE(rycpu_t* __cpu__, size_t __sp__, const T& val, ARGS...args)
{
	CPU_REG_EMPLACE(__cpu__->stack[__sp__], val, args...);
}


typedef void (*cpu_op_t)(rycpu_t *cpu, asmins_t *ins);
typedef void (*rycpu_swi_t)(rycpu_t *cpu, asmins_t *ins);

struct swi_handlers_t {
	const char *name;
	rycpu_swi_t op;
};

struct swi_table_t {
	const char* name;
	swi_handlers_t *handlers;
};


rycpu_t *ry_cpu_create(r_uint_t stacksize, swi_table_t* switables);
rycpu_t *ry_cpu_create_default();
void ry_cpu_destroy(rycpu_t *cpu);
void ry_cpu_switables(rycpu_t *cpu, swi_table_t* switables);
int ry_cpu_exec(rycpu_t *cpu, asmins_t *prog, r_uint_t off, r_uint_t debug);
r_uint_t ry_cpu_swilookup(swi_table_t *table, const char* table_name, const char* swi_name);
void ry_cpu_push(rycpu_t* cpu, const varreg_t& reg);
void ry_cpu_push(rycpu_t* cpu, varreg_t&& reg);
void ry_cpu_pushref(rycpu_t* cpu, RyObject* obj);
void ry_cpu_pushref(rycpu_t* cpu, const RyPointer& obj);
varreg_t ry_cpu_pop(rycpu_t *cpu);
size_t ry_cpu_stackparam_count(rycpu_t* cpu);
varreg_t& ry_cpu_stackparam_get(rycpu_t*cpu, size_t n);

std::string ry_cpu_dump(const asmins_t *pi, r_uint_t count);


#define CPU_OP_TABLE \
	CPU_OP_ENTRY_FIRST(CPU_EXT, cpu_op_ext) 		/* Exit */ \
	CPU_OP_ENTRY(CPU_ABORT, cpu_op_abort) 			/* Abort: set the error code, op1: error code */ \
	CPU_OP_ENTRY(CPU_MOV, cpu_op_mov) 				/* op1 <-- op2 */ \
	CPU_OP_ENTRY(CPU_MOVS, cpu_op_movs) 			/* op1 <-- op2, compare to 0 and set the status accordingly */ \
	CPU_OP_ENTRY(CPU_SWI, cpu_op_swi) 				/* Invoke Software Interrupt handler (table_offset, handler_offset) */ \
	CPU_OP_ENTRY(CPU_PRN, cpu_op_prn) 				/* Debug print the register specified by op1 */ \
	CPU_OP_ENTRY(CPU_ADD, cpu_op_add) 				/* Add: op1 = op2 + op3 */ \
	CPU_OP_ENTRY(CPU_ADDS, cpu_op_adds) 			/* Add: op1 = op2 + op3, update the status register */ \
	CPU_OP_ENTRY(CPU_ADC, cpu_op_adc) 				/* Add: op1 = op2 + op3 + C, update the status register */ \
	CPU_OP_ENTRY(CPU_SUB, cpu_op_sub)				/* Sub: op1 = op2 - op3 */ \
	CPU_OP_ENTRY(CPU_SUBS, cpu_op_subs)				/* Sub: op1 = op2 - op3, update the status register */ \
	CPU_OP_ENTRY(CPU_SBC, cpu_op_sbc)				/* Sub: op1 = op2 - op3 - C, update the status register */ \
	CPU_OP_ENTRY(CPU_MUL, cpu_op_mul)				/* Mul: op1 = op2 * op3 */ \
	CPU_OP_ENTRY(CPU_MLS, cpu_op_mls)				/* Signed Multiplication: op1 = op2 * op3, update the status register */ \
	CPU_OP_ENTRY(CPU_MULS, cpu_op_muls)				/* Mul: op1 = op2 * op3, update the status register*/ \
	CPU_OP_ENTRY(CPU_DIV, cpu_op_div)				/* Divide: op1 = op2 / op3 */ \
	CPU_OP_ENTRY(CPU_DVS, cpu_op_dvs)				/* Signed division: op1 = op2 / op3 */ \
	CPU_OP_ENTRY(CPU_DIVS, cpu_op_divs)				/* Divide: op1 = op2 / op3, Update the status register */ \
	CPU_OP_ENTRY(CPU_MOD, cpu_op_mod)				/* Modulo: op1 = op2 % op3 */ \
	CPU_OP_ENTRY(CPU_MODS, cpu_op_mods)				/* Modulo: op1 = op2 % op3, Update the status register */ \
	CPU_OP_ENTRY(CPU_ASR, cpu_op_asr) 				/* Arithmetic shift right: op1 = op2 >> op3, preserve the signed bit */ \
	CPU_OP_ENTRY(CPU_SWP, cpu_op_swp) 				/* op1 <--> op2 */ \
	CPU_OP_ENTRY(CPU_INC, cpu_op_inc) 				/* INC: op1 = op1 + 1 */ \
	CPU_OP_ENTRY(CPU_DEC, cpu_op_dec) 				/* DEC: op1 = op1 - 1 */ \
	CPU_OP_ENTRY(CPU_LSL, cpu_op_lsl)				/* Bitwise Shift Left: op1 = op2 << op3, update the status register */ \
	CPU_OP_ENTRY(CPU_LSR, cpu_op_lsr)				/* Bitwise Shift Right: op1 = op2 >> op3, update the status register */ \
	CPU_OP_ENTRY(CPU_LSRS, cpu_op_lsrs)				/* Signed Bitwise Shift Right: op1 = op2 >> op3, update the status register */ \
	CPU_OP_ENTRY(CPU_NOT, cpu_op_not)				/* Bitwise NOT: op1 = ~op2, Update the status register */ \
	CPU_OP_ENTRY(CPU_TST, cpu_op_tst) 				/* The same as AND, but doesn't save the result. Update the status register. */ \
	CPU_OP_ENTRY(CPU_TEQ, cpu_op_teq)				/* The same as XOR, but doesn't save the result. Update the status register. */ \
	CPU_OP_ENTRY(CPU_AND, cpu_op_and) 				/* Bitwise AND: op1 = op2 & op3, update status register */ \
	CPU_OP_ENTRY(CPU_ORR, cpu_op_orr)				/* Bitwise OR: op1 = op2 | op3, update the status register */ \
	CPU_OP_ENTRY(CPU_BIC, cpu_op_bic) 				/* Bit Clear: op1 = op2 & ~op3, update status register */ \
	CPU_OP_ENTRY(CPU_CLZ, cpu_op_clz) 				/* Count Leading Zeros: op1 = leading_zeros(op2) */ \
	CPU_OP_ENTRY(CPU_XOR, cpu_op_xor)				/* XOR: op1 = op2 ^ op3, update the status register */ \
	CPU_OP_ENTRY(CPU_BX, cpu_op_bx)					/* Jump to op1 */ \
	CPU_OP_ENTRY(CPU_BXEQ, cpu_op_bxeq)				/* Jump to op1, if equal */ \
	CPU_OP_ENTRY(CPU_BXNEQ, cpu_op_bxneq)			/* Jump to op1, if not equal */ \
	CPU_OP_ENTRY(CPU_BXLEQ, cpu_op_bxleq)			/* Jump if less or equal */ \
	CPU_OP_ENTRY(CPU_BXLES, cpu_op_bxles)			/* Jump if less */ \
	CPU_OP_ENTRY(CPU_BXGEQ, cpu_op_bxgeq)			/* Jump if greater or equal */ \
	CPU_OP_ENTRY(CPU_BXGRE, cpu_op_bxgre)			/* Jump if greater */ \
	CPU_OP_ENTRY(CPU_BXL, cpu_op_bxl)				/* Jump to op1, link */ \
	CPU_OP_ENTRY(CPU_B, cpu_op_b)					/* Branch */ \
	CPU_OP_ENTRY(CPU_BEQ, cpu_op_beq)				/* Branch with op1, if equal */ \
	CPU_OP_ENTRY(CPU_BNEQ, cpu_op_bneq)				/* Branch with op1, if not equal */ \
	CPU_OP_ENTRY(CPU_BLEQ, cpu_op_bleq)				/* Branch with op1, if less or equal */ \
	CPU_OP_ENTRY(CPU_BGEQ, cpu_op_bgeq)				/* Branch with op1, if greater or equal */ \
	CPU_OP_ENTRY(CPU_BLES, cpu_op_bles)				/* Branch with op1, if less */ \
	CPU_OP_ENTRY(CPU_BGRE, cpu_op_bgre)				/* Branch with op1, if less or greater */ \
	CPU_OP_ENTRY(CPU_BL, cpu_op_bl)					/* Branch Link */ \
	CPU_OP_ENTRY(CPU_RET, cpu_op_ret)				/* Return, Set PC to LR */ \
	CPU_OP_ENTRY(CPU_STR, cpu_op_str)				/* Save: val_at_location(op2) = op1 */ \
	CPU_OP_ENTRY(CPU_STRP, cpu_op_strp)				/* Save pointer: pointer_at_location(op2) = op1 */ \
	CPU_OP_ENTRY(CPU_STRB, cpu_op_strb)				/* Save: u8_at_location(op2) = op1 */ \
	CPU_OP_ENTRY(CPU_STRH, cpu_op_strh)				/* Save: u16_at_location(op2) = op1 */ \
	CPU_OP_ENTRY(CPU_STRW, cpu_op_strw)				/* Save: u32_at_location(op2) = op1 */ \
	CPU_OP_ENTRY(CPU_STRR, cpu_op_strr)				/* Save: rycpureg_at_location(op2) = op1 */ \
	CPU_OP_ENTRY(CPU_LDR, cpu_op_ldr)				/* Load: op1 = val_at_location(op2) */ \
	CPU_OP_ENTRY(CPU_LDRP, cpu_op_ldrp)				/* Load pointer: op1 = pointer_at_location(op2) */ \
	CPU_OP_ENTRY(CPU_LDRB, cpu_op_ldrb)				/* Load Byte: op1 = u8_at_location(op2) */ \
	CPU_OP_ENTRY(CPU_LDRH, cpu_op_ldrh)				/* Load Half Word: op1 = u16_at_location(op2) */ \
	CPU_OP_ENTRY(CPU_LDRW, cpu_op_ldrw)				/* Load Word: op1 = u32_at_location(op2) */ \
	CPU_OP_ENTRY(CPU_LDRR, cpu_op_ldrr)				/* Load rycpureg: op1 = rycpureg_at_location(op2) */ \
	CPU_OP_ENTRY(CPU_CFLAG, cpu_op_cflag)			/* Clear flag */ \
	CPU_OP_ENTRY(CPU_CLR, cpu_op_clr)				/* Clear op1 */ \
	CPU_OP_ENTRY(CPU_CLRR, cpu_op_clrr)				/* Clear: rycpureg at memory location op1 */ \
	CPU_OP_ENTRY(CPU_STM, cpu_op_stm)				/* Store multiple */ \
	CPU_OP_ENTRY(CPU_LDM, cpu_op_ldm)				/* Load multiple */ \
	CPU_OP_ENTRY(CPU_STS, cpu_op_sts)				/* Store op1 on the stack at position op2 + op3, i.e. stack[op2 + op3] = op1 */ \
	CPU_OP_ENTRY(CPU_LDS, cpu_op_lds)				/* Load op1 from the stack at position op2 + op3, i.e. op1 = stack[op2 + op3] */ \
	CPU_OP_ENTRY(CPU_PUSH, cpu_op_push)				/* Push op1 on the stack */ \
	CPU_OP_ENTRY(CPU_POP, cpu_op_pop)				/* Pop a value from the stack and save it op1, update SP if op1 is not the SP */ \
	CPU_OP_ENTRY(CPU_PUSHM, cpu_op_pushm)			/* Push multiple */ \
	CPU_OP_ENTRY(CPU_POPM, cpu_op_popm)				/* Pop multiple */ \
	CPU_OP_ENTRY(CPU_CMP, cpu_op_cmp)				/* Compare Positive: status register is updated based on the result: op1 - op2 */ \
	CPU_OP_ENTRY(CPU_CMN, cpu_op_cmn) 				/* Compare Negative: status register is updated based on the result: op1 + op2 */ \
	CPU_OP_ENTRY(CPU_NOP, cpu_op_nop) 				/* NOP instruction. */ \
	CPU_OP_ENTRY(CPU_ROR, cpu_op_ror)				/* Rotate right, the last bit rotated out updates the Carry flag */ \
	CPU_OP_ENTRY(CPU_ADDRS, cpu_op_addrs)			/* Memory location of the stack at offset op2 + op3 */ \
	CPU_OP_ENTRY(RYO_BOOLEAN, cpu_ryo_boolean)		/* Create Boolean RyObject: (stack) = Boolean(op1) */ \
	CPU_OP_ENTRY(RYO_ARRAY, cpu_ryo_array)			/* Create Array RyObject: (stack) = Array() */ \
	CPU_OP_ENTRY(RYO_MAP, cpu_ryo_map)				/* Create Map RyObject on the stack */ \
	CPU_OP_ENTRY(RYO_CLONE, cpu_ryo_clone)			/* Clone RyObject: op1 = Clone(op2) */ \
	CPU_OP_ENTRY(RYO_CALL, cpu_ryo_call)			/* Call a function or SWI */ \
	CPU_OP_ENTRY(RYO_PTOSTR, cpu_ryo_ptostr)		/* Creates StringObject from pointer to str */ \
	CPU_OP_ENTRY(RYO_PTOINT, cpu_ryo_ptoint)		/* Creates IntObject from pointer to int */ \
	CPU_OP_ENTRY(RYO_UTOINT, cpu_ryo_utoint)		/* Creates IntObject from unsigned int */ \
	CPU_OP_ENTRY(RYO_MOVREF, cpu_ryo_movref)		/* Move reference: op1 = ref(op2) */ \
	CPU_OP_ENTRY(RYO_UNREF, cpu_ryo_unref)			/* Convert reference to value: op1, if op1 is not reference, nothing is done. */ \
	CPU_OP_ENTRY(RYO_PTOREF, cpu_ryo_ptoref)		/* Create reference RyObject*: op1 = ref(op2), op2 is of type RyObject* stored as r_uint_t */ \
	CPU_OP_ENTRY(RYO_LDSREF, cpu_ryo_ldsref)		/* Load op1 from the stack at position op2 + op3, i.e. op1 = ref(stack[op2 + op3]) */ \
	CPU_OP_ENTRY(RYO_FUNCTOP, cpu_ryo_functop)		/* Get the pointer to asmins_t of the underlying code: op1 = asmins_ptr(op2) */ \
	CPU_OP_ENTRY(RYO_AT, cpu_ryo_at)				/* Move the RyObject reference to op1. op1 <= op2[op3], op2 is reference pointer to array or map, op3 is index */ \
	CPU_OP_ENTRY(RYO_REF, cpu_ryo_ref)				/* OP1: var map, OP2: number of params on the stack. Use the stack to perform the reference lookup. For example: a[1][2], the stack would have: a,1,2 and the ref would the value of op1(a,1,2) on the stack, after removing a,1,2 */ \
	CPU_OP_ENTRY(RYO_REFEXT, cpu_ryo_refext)		/* OP1: var map 1, OP2: var map 2, OP3: number of params on the stack. Use the stack to perform the reference lookup in two maps. For example: a[1][2], the stack would have: a,1,2 and the ref would the value of op1(a,1,2) on the stack, after removing a,1,2 */ \
	CPU_OP_ENTRY(RYO_ASSAT, cpu_ryo_assignat)		/* Assign the RyObject of op1 to op2[o3]. op1 => op2[op3], op2 is reference pointer to array or map, op3 is index */ \
	CPU_OP_ENTRY(RYO_ASSIGN, cpu_ryo_assign)		/* Use the stack to perform the assignment. For example: a[1][2] = 3, the stack would have: a,1,2,3 and the assigment would do op1(a,1,2) = 3, a,1,2 would be removed from the stack */ \
	CPU_OP_ENTRY(RYO_APPEND, cpu_ryo_append)		/* Append the RyObject of op1 to op2 list. op1 => op2(end), op2 is reference pointer to array */ \
	CPU_OP_ENTRY(RYO_INC, cpu_ryo_inc) 				/* INC: op1->Inc() */ \
	CPU_OP_ENTRY(RYO_DEC, cpu_ryo_dec) 				/* DEC: op1->Dec() */ \
	CPU_OP_ENTRY(RYO_NEG, cpu_ryo_neg)				/* Adds two RyObjects: op1 = -op1 */ \
	CPU_OP_ENTRY(RYO_ADD, cpu_ryo_add)				/* Adds two RyObjects: op1 = op2 + op3 */ \
	CPU_OP_ENTRY(RYO_SUB, cpu_ryo_sub)				/* Subs two RyObjects: op1 = op2 - op3 */ \
	CPU_OP_ENTRY(RYO_MUL, cpu_ryo_mul)				/* Muls two RyObjects: op1 = op2 * op3 */ \
	CPU_OP_ENTRY(RYO_DIV, cpu_ryo_div)				/* Divs two RyObjects: op1 = op2 / op3 */ \
	CPU_OP_ENTRY(RYO_MOD, cpu_ryo_mod)				/* Mods two RyObjects: op1 = op2 % op3 */ \
	CPU_OP_ENTRY(RYO_LSL, cpu_ryo_lsl)				/* Bitwise Shift Left: op1 = op2 << op3, update the status register */ \
	CPU_OP_ENTRY(RYO_LSR, cpu_ryo_lsr)				/* Bitwise Shift Right: op1 = op2 >> op3, update the status register */ \
	CPU_OP_ENTRY(RYO_EQ, cpu_ryo_eq)				/* Compares two RyObjects: op1 = op2 == op3 */ \
	CPU_OP_ENTRY(RYO_NE, cpu_ryo_ne)				/* Compares two RyObjects: op1 = op2 != op3 */ \
	CPU_OP_ENTRY(RYO_LT, cpu_ryo_lt)				/* Compares two RyObjects: op1 = op2 < op3 */ \
	CPU_OP_ENTRY(RYO_GT, cpu_ryo_gt)				/* Compares two RyObjects: op1 = op2 > op3 */ \
	CPU_OP_ENTRY(RYO_LE, cpu_ryo_le)				/* Compares two RyObjects: op1 = op2 <= op3 */ \
	CPU_OP_ENTRY(RYO_GE, cpu_ryo_ge)				/* Compares two RyObjects: op1 = op2 >= op3 */ \



#define RYO_CALL_SAVEDREGS	4


#define CPU_OP_ENTRY_FIRST(op, handler) op = 0,
#define CPU_OP_ENTRY(op, handler) op,

enum {
	CPU_OP_TABLE
};

asmins_t ry_asm(r_uint_t opcode, r_uint_t op1, r_uint_t op2, r_uint_t op3, r_uint_t data);
asmins_t ry_asmc(r_uint_t opcode, r_uint_t op1, r_uint_t op2, r_uint_t op3, r_uint_t data, r_uint_t cond);
void ryreg_set_signed(varreg_t *r, r_int_t val);
varreg_t ryreg_create_signed(r_int_t val);
void ryreg_set_unsigned(varreg_t *r, r_uint_t val);
varreg_t ryreg_create_unsigned(r_uint_t val);
void ryreg_set_pointer(varreg_t *r, r_pointer_t val);
varreg_t ryreg_create_pointer(r_pointer_t val);

}


#endif
