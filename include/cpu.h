#pragma once

#include <stdint.h>

// Definitions

#define CPU_PAUSED_ON_STARTUP
#define CPU_MAX_BREAKPOINTS 8

#define NO_OPERAND       ((cpu_operand_t){.type = operand_no})
#define REG_OPERAND(n)   ((cpu_operand_t){.type = operand_reg,   .reg = n})
#define MEM8_OPERAND(m)  ((cpu_operand_t){.type = operand_mem8,  .mem = m})
#define MEM16_OPERAND(m) ((cpu_operand_t){.type = operand_mem16, .mem = m})
#define IMM8_OPERAND(v)  ((cpu_operand_t){.type = operand_imm8,  .imm8 = v})
#define IMM16_OPERAND(v) ((cpu_operand_t){.type = operand_imm16, .imm16 = v})

#define READ_FLAG(f)     ((regs.flags >> (f)) & 1)
#define WRITE_FLAG(f, v) (regs.flags = (regs.flags & ~(1 << (f))) | (((v) ? 1 : 0) << (f)))

#define W_BITS(w) (w ? 16 : 8)

// Instruction mnemoncs
typedef enum {
    mnem_aaa,   mnem_aad,   mnem_aam,   mnem_aas,   mnem_adc,  mnem_add,   mnem_and,
    mnem_call,  mnem_cbw,   mnem_clc,   mnem_cld,   mnem_cli,  mnem_cmc,   mnem_cmp,
    mnem_cmpsb, mnem_cmpsw, mnem_cwd,   mnem_daa,   mnem_das,  mnem_dec,   mnem_div,
    mnem_idiv,  mnem_imul,  mnem_in,    mnem_inc,   mnem_int,  mnem_into,  mnem_iret,
    mnem_jo,    mnem_jno,   mnem_jc,    mnem_jnc,   mnem_jz,   mnem_jnz,   mnem_jbe,
    mnem_ja,    mnem_js,    mnem_jns,   mnem_jp,    mnem_jnp,  mnem_jl,    mnem_jge,
    mnem_jle,   mnem_jg,    mnem_lahf,  mnem_lds,   mnem_lea,  mnem_loopz, mnem_loopnz,
    mnem_les,   mnem_lodsb, mnem_lodsw, mnem_loop,  mnem_mov,  mnem_movsb, mnem_movsw,
    mnem_mul,   mnem_neg,   mnem_nop,   mnem_or,    mnem_out,  mnem_pop,
    mnem_popf,  mnem_push,  mnem_pushf, mnem_rcl,  mnem_rcr,   mnem_ret,
    mnem_retf,  mnem_rol,   mnem_ror,   mnem_sahf,  mnem_not,  mnem_sar,   mnem_sbb,
    mnem_scasb, mnem_scasw, mnem_shl,   mnem_shr,   mnem_stc,  mnem_std,   mnem_sti,
    mnem_stosb, mnem_stosw, mnem_sub,   mnem_test,  mnem_xchg, mnem_xlatb, mnem_xor,
    mnem_wait,  mnem_jcxz,  mnem_jmp,   mnem_hlt
} cpu_mnem_t;

// Instruction operands
typedef enum {
    operand_no, operand_reg, operand_mem8, operand_mem16, operand_imm8, operand_imm16,
    operand_far_at_location
} cpu_oper_type_t;

// Registers that can be operands
typedef enum {
    reg_ax, reg_al, reg_cx, reg_cl,
    reg_dx, reg_dl, reg_bx, reg_bl,
    reg_sp, reg_ah, reg_bp, reg_ch,
    reg_si, reg_dh, reg_di, reg_bh,
    reg_cs, reg_ds, reg_ss, reg_es
} cpu_reg_oper_t;

// Modes for memory operands
typedef enum {
    mm_bx_si, mm_bx_di, mm_bp_si, mm_bp_di, mm_si, mm_di, mm_zero, mm_bp, mm_bx
} cpu_mem_mode_t;

// Memory operand displacements
typedef enum {
    disp_no, disp_8, disp_16, disp_far, disp_abs
} cpu_mem_disp_t;

// Memory operand
typedef struct {
    cpu_mem_mode_t mode;
    cpu_mem_disp_t disp;
    union {
        uint16_t disp16;
        uint8_t disp8;
        struct {
            uint16_t far_segm;
            uint16_t far_offs;
        };
    };
} cpu_mem_oper_t;

// Operand
typedef struct {
    cpu_oper_type_t type;
    union {
        cpu_reg_oper_t reg;
        cpu_mem_oper_t mem;
        uint8_t imm8;
        uint16_t imm16;
    };
} cpu_operand_t;

// Segment overrides
typedef enum {
    so_ds, so_cs, so_ss, so_es
} cpu_segm_override_t;

// REP prefixes
typedef enum {
    rp_no, rp_rep, rp_repne
} cpu_rep_prefix_t;

// Instruction
typedef struct {
    // meta
    uint8_t length, valid:1;

    // prefixes
    uint8_t lock:1;
    cpu_rep_prefix_t rp;
    cpu_segm_override_t so;

    // instruction itself
    cpu_mnem_t mnemonic;
    cpu_operand_t oper1, oper2;
} cpu_instr_t;

// CPU state
#define FLAG_CF 0
#define FLAG_PF 2
#define FLAG_AF 4
#define FLAG_ZF 6
#define FLAG_SF 7
#define FLAG_TF 8
#define FLAG_IF 9
#define FLAG_DF 10
#define FLAG_OF 11
typedef struct {
    union {
        uint16_t ax;
        struct __attribute__((packed)) {
            uint8_t al, ah;
        };
    };
    union {
        uint16_t bx;
        struct __attribute__((packed)) {
            uint8_t bl, bh;
        };
    };
    union {
        uint16_t cx;
        struct __attribute__((packed)) {
            uint8_t cl, ch;
        };
    };
    union {
        uint16_t dx;
        struct __attribute__((packed)) {
            uint8_t dl, dh;
        };
    };
    uint16_t si, di, bp, sp;
    uint16_t cs, ds, es, ss;
    uint16_t ip;
    uint16_t flags;
} cpu_regs_t;

// Breakpoint
typedef struct {
    uint8_t active;
    uint16_t cs;
    uint16_t ip;
} cpu_brkpnt_t;

// Private functions

uint16_t _cpu_read16 (uint32_t* addr);
void     _cpu_write16(uint32_t* addr, uint16_t val);

// Public functions

void cpu_reset(void);
void cpu_step (void);
void cpu_loop (void);

cpu_instr_t cpu_fetch_decode(uint32_t addr);
void        cpu_instr_sprint(cpu_instr_t instr, char* buf);
void        cpu_disasm      (uint32_t start, int32_t len);

void       cpu_print_state     (void);
void       cpu_set_running     (uint8_t val);
void       cpu_single_step     (uint32_t steps);
uint8_t    cpu_running         (void);
void       cpu_set_trace       (uint8_t val);
uint8_t    cpu_trace           (void);
uint8_t    cpu_breakpoint      (uint16_t cs, uint16_t ip);
void       cpu_del_breakpoint  (uint8_t num);
void       cpu_list_breakpoints(void);
cpu_regs_t cpu_state           (void);

void cpu_repl_init(void);