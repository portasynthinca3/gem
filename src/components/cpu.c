// Resources I used:
// General encoding description: http://aturing.umcs.maine.edu/~meadow/courses/cos335/8086-instformat.pdf
// Opcode map: http://mlsite.net/8086/
// Operation: https://dsearls.org/courses/C391OrgSys/IntelAL/8086_instruction_set.html#JC

#include "cpu.h"
#include "machine.h"
#include <string.h>
#include <esp_log.h>
#include <stdlib.h>
#include <stdio.h>

#define TAG "8086-exec"

// CPU state

cpu_regs_t regs;

// Lookup tables
uint8_t modrm_tab_reg_w0[8] = {
    reg_al, reg_cl, reg_dl, reg_bl,
    reg_ah, reg_ch, reg_dh, reg_bh
};
uint8_t modrm_tab_reg_w1[8] = {
    reg_ax, reg_cx, reg_dx, reg_bx,
    reg_sp, reg_bp, reg_si, reg_di
};
uint8_t modrm_tab_sreg[8] = {
    reg_es, reg_cs, reg_ss, reg_ds
};
uint8_t modrm_tab1[8] = {
    mm_bx_si, mm_bx_di, mm_bp_si, mm_bp_di, mm_si, mm_di, mm_zero, mm_bx
};
uint8_t modrm_tab2[8] = {
    mm_bx_si, mm_bx_di, mm_bp_si, mm_bp_di, mm_si, mm_di, mm_bp, mm_bx
};
cpu_reg_oper_t unary_op_tab[8] = {
    reg_ax, reg_cx, reg_dx, reg_bx, reg_sp, reg_bp, reg_si, reg_di
};
cpu_mnem_t cond_jmp_tab[16] = {
    mnem_jo, mnem_jno, mnem_jc, mnem_jnc, mnem_jz, mnem_jnz, mnem_jbe, mnem_ja,
    mnem_js, mnem_jns, mnem_jp, mnem_jnp, mnem_jl, mnem_jge, mnem_jle, mnem_jg
};
cpu_mnem_t grp1_tab[8] = {
    mnem_add, mnem_or, mnem_adc, mnem_sbb, mnem_and, mnem_sub, mnem_xor, mnem_cmp
};
cpu_mnem_t grp2_tab[8] = {
    mnem_rol, mnem_ror, mnem_rcl, mnem_rcr, mnem_shl, mnem_shr, 0, mnem_sar
};
cpu_mnem_t grp3_tab[8] = {
    mnem_test, mnem_not, mnem_neg, mnem_mul, mnem_imul, mnem_div, mnem_idiv
};
cpu_reg_oper_t reg_to_sreg_tab[4] = {
    reg_es, reg_cs, reg_ss, reg_ds
};
cpu_reg_oper_t xchg_tab[7] = {
    reg_cx, reg_dx, reg_bx, reg_sp, reg_bp, reg_si, reg_di
};
cpu_mnem_t str_tab[10] = {
    mnem_movsb, mnem_movsw, mnem_cmpsb, mnem_cmpsw, mnem_stosb, mnem_stosw,
    mnem_lodsb, mnem_lodsw, mnem_scasb, mnem_scasw
};
cpu_mnem_t loop_tab[4] = {
    mnem_loopnz, mnem_loopz, mnem_loop, mnem_jcxz
};
cpu_mnem_t flag_tab[6] = {
    mnem_clc, mnem_stc, mnem_cli, mnem_sti, mnem_cld, mnem_std
};

char* reg_names[20] = {
    "ax", "al", "cx", "cl", "dx", "dl", "bx", "bl",
    "sp", "ah", "bp", "ch", "si", "dh", "di", "bh",
    "cs", "ds", "ss", "es"
};
char* sreg_names[4] = {
    "ds", "cs", "ss", "es"
};
char* disp_names[9] = {
    "bx+si", "bx+di", "bp+si", "bp+di", "si", "di", "0", "bp", "bx"
};
char* instr_names[] = {
    "aaa",   "aad",   "aam",   "aas",   "adc",  "add",   "and",
    "call",  "cbw",   "clc",   "cld",   "cli",  "cmc",   "cmp",
    "cmpsb", "cmpsw", "cwd",   "daa",   "das",  "dec",   "div",
    "idiv",  "imul",  "in",    "inc",   "int",  "into",  "iret",
    "jo",    "jno",   "jc",    "jnc",   "jz",   "jnz",   "jbe",
    "ja",    "js",    "jns",   "jp",    "jnp",  "jl",    "jge",
    "jle",   "jg",    "lahf",  "lds",   "lea",  "loopz", "loopnz",
    "les",   "lodsb", "lodsw", "loop",  "mov",  "movsb", "movsw",
    "mul",   "neg",   "nop",   "or",    "out",  "pop",   "popa",
    "popf",  "push",  "pusha", "pushf", "rcl",  "rcr",   "ret",
    "retf",  "rol",   "ror",   "sahf",  "sal",  "sar",   "sbb",
    "scasb", "scasw", "shl",   "shr",   "stc",  "std",   "sti",
    "stosb", "stosw", "sub",   "test",  "xchg", "xlatb", "xor",
    "wait",  "jcxz",  "jmp",   "hlt",   "not"
};

// Private functions

uint16_t _cpu_read16(uint32_t* addr) {
    uint16_t val = READ((*addr)++);
    val |= (uint16_t)READ((*addr)++) << 8;
    return val;
}
void _cpu_write16(uint32_t addr, uint16_t val) {
    WRITE(addr, val & 0xff);
    WRITE(addr + 1, val >> 8);
}

cpu_mem_oper_t _cpu_decode_far(uint32_t* addr) {
    uint16_t offs = _cpu_read16(addr);
    uint16_t segm = _cpu_read16(addr);

    return (cpu_mem_oper_t){.disp = disp_far, .far_offs = offs, .far_segm = segm};
}

cpu_mem_oper_t _cpu_decode_abs(uint32_t* addr) {
    uint16_t offs = _cpu_read16(addr);

    return (cpu_mem_oper_t){.disp = disp_abs, .disp16 = offs};
}

// printing functions
void _cpu_mem_sprint(cpu_mem_oper_t oper, char* buf, cpu_segm_override_t so) {
    switch(oper.disp) {
        case disp_abs:
            sprintf(buf, "[%s:0x%04x]", sreg_names[so], oper.disp16);
            break;
        case disp_far:
            sprintf(buf, "0x%04x:0x%04x", oper.far_segm, oper.far_offs);
            break;
        case disp_no:
            sprintf(buf, "[%s:%s]", sreg_names[so], disp_names[oper.mode]);
            break;
        case disp_8:
            sprintf(buf, "[%s:%s+0x%02x]", sreg_names[so], disp_names[oper.mode], (uint32_t)oper.disp8);
            break;
        case disp_16:
            sprintf(buf, "[%s:%s+0x%04x]", sreg_names[so], disp_names[oper.mode], (uint32_t)oper.disp16);
            break;
    }
}
void _cpu_oper_sprint(cpu_operand_t oper, char* buf, cpu_segm_override_t so) {
    switch(oper.type) {
        case operand_no:
            strcpy(buf, "");
            break;
        case operand_reg:
            strcpy(buf, reg_names[oper.reg]);
            break;
        case operand_imm8:
            sprintf(buf, "%d", (uint32_t)oper.imm8);
            break;
        case operand_imm16:
            sprintf(buf, "%d", (uint32_t)oper.imm16);
            break;
        case operand_mem8:
        case operand_mem16:
            buf += sprintf(buf, "%s ", oper.type == operand_mem8 ? "byte" : "word");
            _cpu_mem_sprint(oper.mem, buf, so);
            break;
        case operand_far_at_location:
            buf += sprintf(buf, "jmptbl ");
            _cpu_mem_sprint(oper.mem, buf, so);
            break;
    }
}
void _cpu_byte_sprint(uint32_t addr, uint32_t len, char* buf) {
    while(len--)
        buf += sprintf(buf, "%02x ", READ(addr++));
}

void _cpu_decode_modrm(uint32_t* addr, uint8_t d, uint8_t w, cpu_operand_t* op1, cpu_operand_t* op2) {
    uint8_t raw = READ((*addr)++);
    uint8_t mod = raw >> 6;
    uint8_t reg = (raw >> 3) & 7;
    uint8_t rm = raw & 7;

    // decode REG
    op1->type = operand_reg;
    op1->reg = (w ? modrm_tab_reg_w1 : modrm_tab_reg_w0)[reg];

    // decode mod+r/m
    switch(mod) {
        case 0:
            op2->type = w ? operand_mem16 : operand_mem8;
            op2->mem.mode = modrm_tab1[rm];
            if(op2->mem.mode == mm_zero) {
                op2->mem.disp = disp_16;
                op2->mem.disp16 = _cpu_read16(addr);
            } else {
                op2->mem.disp = disp_no;
            }
            break;
        case 1:
            op2->type = w ? operand_mem16 : operand_mem8;
            op2->mem.mode = modrm_tab2[rm];
            op2->mem.disp = disp_8;
            op2->mem.disp8 = READ((*addr)++);
            break;
        case 2:
            op2->type = w ? operand_mem16 : operand_mem8;
            op2->mem.mode = modrm_tab2[rm];
            op2->mem.disp = disp_16;
            op2->mem.disp16 = _cpu_read16(addr);
            break;
        case 3:
            op2->type = operand_reg;
            op2->reg = (w ? modrm_tab_reg_w1 : modrm_tab_reg_w0)[rm];
    }

    // swap operands around (r/m <- reg instead of reg <- r/m)
    if(!d) {
        cpu_operand_t temp = *op1;
        *op1 = *op2;
        *op2 = temp;
    }
}

inline uint32_t _cpu_sreg_base(cpu_segm_override_t so) {
    switch(so) {
        case so_ds: return (uint32_t)regs.ds << 4;
        case so_cs: return (uint32_t)regs.cs << 4;
        case so_ss: return (uint32_t)regs.ss << 4;
        case so_es: return (uint32_t)regs.es << 4;
        default:
            ESP_LOGE(TAG, "invalid segment override");
            return 0;
    }
}
inline uint16_t _cpu_index(cpu_mem_mode_t mode) {
    switch(mode) {
        case mm_zero:  return 0;
        case mm_bp:    return regs.bp;
        case mm_bp_di: return regs.bp + regs.di;
        case mm_bp_si: return regs.bp + regs.si;
        case mm_bx:    return regs.bx;
        case mm_bx_di: return regs.bx + regs.di;
        case mm_bx_si: return regs.bx + regs.si;
        case mm_di:    return regs.di;
        case mm_si:    return regs.si;
        default:
            ESP_LOGE(TAG, "invalid index mode");
            return 0;
    }
}
uint32_t _cpu_effective_addr(cpu_mem_oper_t op, cpu_segm_override_t so) {
    switch(op.disp) {
        case disp_abs: return _cpu_sreg_base(so) + op.disp16;
        case disp_far: return ((uint32_t)op.far_segm << 4) + op.far_offs;
        case disp_no:  return _cpu_sreg_base(so) + _cpu_index(op.mode);
        case disp_8:   return _cpu_sreg_base(so) + _cpu_index(op.mode) + *(int8_t*)&op.disp8;
        case disp_16:  return _cpu_sreg_base(so) + _cpu_index(op.mode) + *(int16_t*)&op.disp16;
        default:
            ESP_LOGE(TAG, "invalid displacement");
            return 0;
    }
}

int32_t _cpu_oprd8(cpu_operand_t op, cpu_segm_override_t so) {
    switch(op.type) {
        case operand_no:
            ESP_LOGE(TAG, "tried to read non-existent operand");
            return 0;
        case operand_reg:
            switch(op.reg) {
                case reg_al: return regs.al;
                case reg_bl: return regs.bl;
                case reg_cl: return regs.cl;
                case reg_dl: return regs.dl;
                case reg_ah: return regs.ah;
                case reg_bh: return regs.bh;
                case reg_ch: return regs.ch;
                case reg_dh: return regs.dh;
                default:
                    ESP_LOGE(TAG, "wrong register size");
                    return 0;
            }
        case operand_imm8:
            return op.imm8;
        case operand_imm16:
            ESP_LOGE(TAG, "wrong immediate size");
            return 0;
        case operand_mem8:
            return READ(_cpu_effective_addr(op.mem, so));
        case operand_mem16:
            ESP_LOGE(TAG, "wrong memory size");
            return 0;
        default:
            ESP_LOGE(TAG, "illegal operand type");
            return 0;
    }
}
int32_t _cpu_oprd16(cpu_operand_t op, cpu_segm_override_t so) {
    switch(op.type) {
        case operand_no:
            ESP_LOGE(TAG, "tried to read non-existent operand");
            return 0;
        case operand_reg:
            switch(op.reg) {
                case reg_ax: return regs.ax;
                case reg_bx: return regs.bx;
                case reg_cx: return regs.cx;
                case reg_dx: return regs.dx;
                case reg_si: return regs.si;
                case reg_di: return regs.di;
                case reg_bp: return regs.bp;
                case reg_sp: return regs.sp;
                default:
                    ESP_LOGE(TAG, "wrong register size");
                    return 0;
            }
        case operand_imm8:
            ESP_LOGE(TAG, "wrong immediate size");
            return 0;
        case operand_imm16:
            return op.imm16;
        case operand_mem8:
            ESP_LOGE(TAG, "wrong memory size");
            return 0;
        case operand_mem16: {
            uint32_t addr = _cpu_effective_addr(op.mem, so);
            return _cpu_read16(&addr);
        }
        default:
            ESP_LOGE(TAG, "illegal operand type");
            return 0;
    }
}
int32_t _cpu_opwr8(cpu_operand_t op, cpu_segm_override_t so, int32_t val) {
    switch(op.type) {
        case operand_no:
            ESP_LOGE(TAG, "tried to write to non-existent operand");
            return 0;
        case operand_reg:
            switch(op.reg) {
                case reg_al: return regs.al = val;
                case reg_bl: return regs.bl = val;
                case reg_cl: return regs.cl = val;
                case reg_dl: return regs.dl = val;
                case reg_ah: return regs.ah = val;
                case reg_bh: return regs.bh = val;
                case reg_ch: return regs.ch = val;
                case reg_dh: return regs.dh = val;
                default:
                    ESP_LOGE(TAG, "wrong register size");
                    return 0;
            }
        case operand_imm8:
        case operand_imm16:
            ESP_LOGE(TAG, "tried to write to immediate");
            return 0;
        case operand_mem8:
            WRITE(_cpu_effective_addr(op.mem, so), val);
            return val;
        case operand_mem16:
            ESP_LOGE(TAG, "wrong memory size");
            return 0;
        default:
            ESP_LOGE(TAG, "illegal operand type");
            return 0;
    }
}
int32_t _cpu_opwr16(cpu_operand_t op, cpu_segm_override_t so, int32_t val) {
    switch(op.type) {
        case operand_no:
            ESP_LOGE(TAG, "tried to read non-existent operand");
            return 0;
        case operand_reg:
            switch(op.reg) {
                case reg_ax: return regs.ax = val;
                case reg_bx: return regs.bx = val;
                case reg_cx: return regs.cx = val;
                case reg_dx: return regs.dx = val;
                case reg_si: return regs.si = val;
                case reg_di: return regs.di = val;
                case reg_bp: return regs.bp = val;
                case reg_sp: return regs.sp = val;
                default:
                    ESP_LOGE(TAG, "wrong register size");
                    return 0;
            }
        case operand_imm8:
        case operand_imm16:
            ESP_LOGE(TAG, "tried to write to immediate");
            return 0;
        case operand_mem8:
            ESP_LOGE(TAG, "wrong memory size");
            return 0;
        case operand_mem16:
            _cpu_write16(_cpu_effective_addr(op.mem, so), val);
            return val;
        default:
            ESP_LOGE(TAG, "illegal operand type");
            return 0;
    }
}

inline uint8_t _cpu_parity(uint8_t w, uint32_t val) {
    uint8_t ones = 0;
    for(int i = 0; i < (w ? 16 : 8); i++) {
        if(val & 1) ones++;
        val >>= 1;
    }
    return ones % 2;
}
inline void _cpu_set_fl(uint8_t w, int32_t result) {
    WRITE_FLAG(FLAG_OF, w ? (result != *(int16_t*)&result) : (result != *(int8_t*)&result));
    WRITE_FLAG(FLAG_SF, result < 0);
    WRITE_FLAG(FLAG_ZF, result == 0);
    // TODO: FLAG_AF
    WRITE_FLAG(FLAG_CF, w ? (result & 0x10000) : (result & 0x100));
    WRITE_FLAG(FLAG_PF, _cpu_parity(w, *(uint32_t*)&result));
}

// Public functions

void cpu_reset(void) {
    // reset registers
    memset(&regs, 0, sizeof(regs));
    regs.cs = 0xffff;
}

cpu_instr_t cpu_fetch_decode(uint32_t addr) {
    uint32_t addr_start = addr;
    cpu_instr_t instr;
    memset(&instr, 0, sizeof(instr));
    instr.valid = 1;

    // read prefixes until an opcode is met
    uint8_t op;
    uint8_t prefix = 1;
    do {
        op = READ(addr++);
        switch(op) {
            case 0xf3: instr.rep   = 1;  break;
            case 0xf2: instr.repne = 1;  break;
            case 0xf0: instr.lock  = 1;  break;
            case 0x2e: instr.so = so_cs; break; //
            case 0x3e: instr.so = so_ds; break; // segment overrides
            case 0x26: instr.so = so_es; break; //
            case 0x36: instr.so = so_ss; break; //
            default: prefix = 0;
        }
    } while(prefix);
    // split out bits D and W
    uint8_t d = (op >> 1) & 1;
    uint8_t w = op & 1;

    // check for simultaneous REP and REPNE
    uint8_t rep_cnt = instr.rep + instr.repne;
    if(rep_cnt > 1) instr.valid = 0;

    // 500-line switch incoming!!!!
    switch(op) {
        case 0x00: // add r/m8, r8
        case 0x01: // add r/m16, r16
        case 0x02: // add r8, r/m8
        case 0x03: // add r16, r/m16
            instr.mnemonic = mnem_add;
            _cpu_decode_modrm(&addr, d, w, &instr.oper1, &instr.oper2);
            break;
        case 0x04: // add al, imm8
            instr.mnemonic = mnem_add;
            instr.oper1 = REG_OPERAND(reg_al);
            instr.oper2 = (cpu_operand_t){.type = operand_imm8, .imm8 = READ(addr++)};
            break;
        case 0x05: // add ax, imm16
            instr.mnemonic = mnem_add;
            instr.oper1 = REG_OPERAND(reg_ax);
            instr.oper2 = (cpu_operand_t){.type = operand_imm16, .imm16 = READ(addr++)};
            instr.oper2.imm16 |= (uint16_t)READ(addr++) << 8;
            break;
        case 0x06: // push es
            instr.mnemonic = mnem_push;
            instr.oper1 = REG_OPERAND(reg_es);
            instr.oper2 = NO_OPERAND;
            break;
        case 0x07: // pop es
            instr.mnemonic = mnem_pop;
            instr.oper1 = REG_OPERAND(reg_es);
            instr.oper2 = NO_OPERAND;
            break;
        case 0x08: // or r/m8, r8
        case 0x09: // or r/m16, r16
        case 0x0a: // or r8, r/m8
        case 0x0b: // or r16, r/m16
            instr.mnemonic = mnem_or;
            _cpu_decode_modrm(&addr, d, w, &instr.oper1, &instr.oper2);
            break;
        case 0x0c: // or al, imm8
            instr.mnemonic = mnem_or;
            instr.oper1 = REG_OPERAND(reg_al);
            instr.oper2 = (cpu_operand_t){.type = operand_imm8, .imm8 = READ(addr++)};
            break;
        case 0x0d: // or ax, imm16
            instr.mnemonic = mnem_or;
            instr.oper1 = REG_OPERAND(reg_ax);
            instr.oper2 = (cpu_operand_t){.type = operand_imm16, .imm16 = READ(addr++)};
            instr.oper2.imm16 |= (uint16_t)READ(addr++) << 8;
            break;
        case 0x0e: // push cs
            instr.mnemonic = mnem_push;
            instr.oper1 = REG_OPERAND(reg_cs);
            instr.oper2 = NO_OPERAND;
            break;

        case 0x10: // adc r/m8, r8
        case 0x11: // adc r/m16, r16
        case 0x12: // adc r8, r/m8
        case 0x13: // adc r16, r/m16
            instr.mnemonic = mnem_adc;
            _cpu_decode_modrm(&addr, d, w, &instr.oper1, &instr.oper2);
            break;
        case 0x14: // adc al, imm8
            instr.mnemonic = mnem_adc;
            instr.oper1 = REG_OPERAND(reg_al);
            instr.oper2 = (cpu_operand_t){.type = operand_imm8, .imm8 = READ(addr++)};
            break;
        case 0x15: // adc ax, imm16
            instr.mnemonic = mnem_adc;
            instr.oper1 = REG_OPERAND(reg_ax);
            instr.oper2 = (cpu_operand_t){.type = operand_imm16, .imm16 = READ(addr++)};
            instr.oper2.imm16 |= (uint16_t)READ(addr++) << 8;
            break;
        case 0x16: // push ss
            instr.mnemonic = mnem_push;
            instr.oper1 = REG_OPERAND(reg_ss);
            instr.oper2 = NO_OPERAND;
            break;
        case 0x17: // pop ss
            instr.mnemonic = mnem_pop;
            instr.oper1 = REG_OPERAND(reg_ss);
            instr.oper2 = NO_OPERAND;
            break;
        case 0x18: // sbb r/m8, r8
        case 0x19: // sbb r/m16, r16
        case 0x1a: // sbb r8, r/m8
        case 0x1b: // sbb r16, r/m16
            instr.mnemonic = mnem_sbb;
            _cpu_decode_modrm(&addr, d, w, &instr.oper1, &instr.oper2);
            break;
        case 0x1c: // sbb al, imm8
            instr.mnemonic = mnem_sbb;
            instr.oper1 = REG_OPERAND(reg_al);
            instr.oper2 = (cpu_operand_t){.type = operand_imm8, .imm8 = READ(addr++)};
            break;
        case 0x1d: // sbb ax, imm16
            instr.mnemonic = mnem_sbb;
            instr.oper1 = REG_OPERAND(reg_ax);
            instr.oper2 = (cpu_operand_t){.type = operand_imm16, .imm16 = READ(addr++)};
            instr.oper2.imm16 |= (uint16_t)READ(addr++) << 8;
            break;
        case 0x1e: // push ds
            instr.mnemonic = mnem_push;
            instr.oper1 = REG_OPERAND(reg_ds);
            instr.oper2 = NO_OPERAND;
            break;
        case 0x1f: // pop ds
            instr.mnemonic = mnem_pop;
            instr.oper1 = REG_OPERAND(reg_ds);
            instr.oper2 = NO_OPERAND;
            break;

        case 0x20: // and r/m8, r8
        case 0x21: // and r/m16, r16
        case 0x22: // and r8, r/m8
        case 0x23: // and r16, r/m16
            instr.mnemonic = mnem_and;
            _cpu_decode_modrm(&addr, d, w, &instr.oper1, &instr.oper2);
            break;
        case 0x24: // and al, imm8
            instr.mnemonic = mnem_and;
            instr.oper1 = REG_OPERAND(reg_al);
            instr.oper2 = (cpu_operand_t){.type = operand_imm8, .imm8 = READ(addr++)};
            break;
        case 0x25: // and ax, imm16
            instr.mnemonic = mnem_and;
            instr.oper1 = REG_OPERAND(reg_ax);
            instr.oper2 = (cpu_operand_t){.type = operand_imm16, .imm16 = READ(addr++)};
            instr.oper2.imm16 |= (uint16_t)READ(addr++) << 8;
            break;
        case 0x26: // es prefix
            ESP_LOGE(TAG, "unexpected ES prefix");
            instr.valid = 0;
            break;
        case 0x27: // daa
            instr.mnemonic = mnem_daa;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0x28: // sub r/m8, r8
        case 0x29: // sub r/m16, r16
        case 0x2a: // sub r8, r/m8
        case 0x2b: // sub r16, r/m16
            instr.mnemonic = mnem_sub;
            _cpu_decode_modrm(&addr, d, w, &instr.oper1, &instr.oper2);
            break;
        case 0x2c: // sub al, imm8
            instr.mnemonic = mnem_sub;
            instr.oper1 = REG_OPERAND(reg_al);
            instr.oper2 = (cpu_operand_t){.type = operand_imm8, .imm8 = READ(addr++)};
            break;
        case 0x2d: // sub ax, imm16
            instr.mnemonic = mnem_sub;
            instr.oper1 = REG_OPERAND(reg_ax);
            instr.oper2 = (cpu_operand_t){.type = operand_imm16, .imm16 = READ(addr++)};
            instr.oper2.imm16 |= (uint16_t)READ(addr++) << 8;
            break;
        case 0x2e: // cs prefix
            ESP_LOGE(TAG, "unexpected CS prefix");
            instr.valid = 0;
            break;
        case 0x2f: // das
            instr.mnemonic = mnem_das;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;

        case 0x30: // xor r/m8, r8
        case 0x31: // xor r/m16, r16
        case 0x32: // xor r8, r/m8
        case 0x33: // xor r16, r/m16
            instr.mnemonic = mnem_xor;
            _cpu_decode_modrm(&addr, d, w, &instr.oper1, &instr.oper2);
            break;
        case 0x34: // xor al, imm8
            instr.mnemonic = mnem_xor;
            instr.oper1 = REG_OPERAND(reg_al);
            instr.oper2 = (cpu_operand_t){.type = operand_imm8, .imm8 = READ(addr++)};
            break;
        case 0x35: // xor ax, imm16
            instr.mnemonic = mnem_xor;
            instr.oper1 = REG_OPERAND(reg_ax);
            instr.oper2 = (cpu_operand_t){.type = operand_imm16, .imm16 = READ(addr++)};
            instr.oper2.imm16 |= (uint16_t)READ(addr++) << 8;
            break;
        case 0x36: // ss prefix
            ESP_LOGE(TAG, "unexpected SS prefix");
            instr.valid = 0;
            break;
        case 0x37: // aaa
            instr.mnemonic = mnem_aaa;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0x38: // cmp r/m8, r8
        case 0x39: // cmp r/m16, r16
        case 0x3a: // cmp r8, r/m8
        case 0x3b: // cmp r16, r/m16
            instr.mnemonic = mnem_cmp;
            _cpu_decode_modrm(&addr, d, w, &instr.oper1, &instr.oper2);
            break;
        case 0x3c: // cmp al, imm8
            instr.mnemonic = mnem_cmp;
            instr.oper1 = REG_OPERAND(reg_al);
            instr.oper2 = (cpu_operand_t){.type = operand_imm8, .imm8 = READ(addr++)};
            break;
        case 0x3d: // cmp ax, imm16
            instr.mnemonic = mnem_cmp;
            instr.oper1 = REG_OPERAND(reg_ax);
            instr.oper2 = (cpu_operand_t){.type = operand_imm16, .imm16 = READ(addr++)};
            instr.oper2.imm16 |= (uint16_t)READ(addr++) << 8;
            break;
        case 0x3e: // ds prefix
            ESP_LOGE(TAG, "unexpected DS prefix");
            instr.valid = 0;
            break;
        case 0x3f: // aas
            instr.mnemonic = mnem_aas;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;

        case 0x40: // inc ax
        case 0x41: // inc cx
        case 0x42: // inc dx
        case 0x43: // inc bx
        case 0x44: // inc sp
        case 0x45: // inc bp
        case 0x46: // inc si
        case 0x47: // inc di
            instr.mnemonic = mnem_inc;
            instr.oper1 = REG_OPERAND(unary_op_tab[op - 0x40]);
            instr.oper2 = NO_OPERAND;
            break;
        case 0x48: // dec ax
        case 0x49: // dec cx
        case 0x4a: // dec dx
        case 0x4b: // dec bx
        case 0x4c: // dec sp
        case 0x4d: // dec bp
        case 0x4e: // dec si
        case 0x4f: // dec di
            instr.mnemonic = mnem_dec;
            instr.oper1 = REG_OPERAND(unary_op_tab[op - 0x48]);
            instr.oper2 = NO_OPERAND;
            break;

        case 0x50: // push ax
        case 0x51: // push cx
        case 0x52: // push dx
        case 0x53: // push bx
        case 0x54: // push sp
        case 0x55: // push bp
        case 0x56: // push si
        case 0x57: // push di
            instr.mnemonic = mnem_push;
            instr.oper1 = REG_OPERAND(unary_op_tab[op - 0x50]);
            instr.oper2 = NO_OPERAND;
            break;
        case 0x58: // pop ax
        case 0x59: // pop cx
        case 0x5a: // pop dx
        case 0x5b: // pop bx
        case 0x5c: // pop sp
        case 0x5d: // pop bp
        case 0x5e: // pop si
        case 0x5f: // pop di
            instr.mnemonic = mnem_pop;
            instr.oper1 = REG_OPERAND(unary_op_tab[op - 0x58]);
            instr.oper2 = NO_OPERAND;
            break;

        case 0x70: // jo  rel8
        case 0x71: // jno rel8
        case 0x72: // jc  rel8
        case 0x73: // jnc rel8
        case 0x74: // jz  rel8
        case 0x75: // jnz rel8
        case 0x76: // jbe rel8
        case 0x77: // ja  rel8
        case 0x78: // jo  rel8
        case 0x79: // jno rel8
        case 0x7a: // js  rel8
        case 0x7b: // jns rel8
        case 0x7c: // jp  rel8
        case 0x7d: // jnp rel8
        case 0x7e: // jl  rel8
        case 0x7f: // jge rel8
            instr.mnemonic = cond_jmp_tab[op - 0x70];
            instr.oper1 = (cpu_operand_t){.type = operand_imm8, .imm8 = READ(addr++)};
            instr.oper2 = NO_OPERAND;
            break;

        case 0x80: // <alu> r/m8,  imm8
        case 0x81: // <alu> r/m16, imm16
        case 0x82: // <alu> r/m8,  imm8 (second one? why?)
        case 0x83: // <alu> r/m16, imm8
            _cpu_decode_modrm(&addr, 1, w, &instr.oper1, &instr.oper2);
            // some wizardry to extract the operation (because of course they had to put it into REG in ModR/M)
            instr.mnemonic = grp1_tab[instr.oper1.reg / 2];
            // read immediate
            instr.oper1 = instr.oper2;
            instr.oper2 = (cpu_operand_t){.type = operand_imm8, .imm8 = READ(addr++)};
            break;
        case 0x84: // test reg8,  r/m8
        case 0x85: // test reg16, r/m16
            instr.mnemonic = mnem_test;
            _cpu_decode_modrm(&addr, 1, w, &instr.oper1, &instr.oper2);
            break;
        case 0x86: // xchg reg8,  r/m8
        case 0x87: // xchg reg16, r/m16
            instr.mnemonic = mnem_xchg;
            _cpu_decode_modrm(&addr, 1, w, &instr.oper1, &instr.oper2);
            break;
        case 0x88: // mov r/m8,  reg8
        case 0x89: // mov r/m16, reg16
        case 0x8a: // mov reg8,  r/m8
        case 0x8b: // mov reg16, r/m16
            instr.mnemonic = mnem_mov;
            _cpu_decode_modrm(&addr, d, w, &instr.oper1, &instr.oper2);
            break;
        case 0x8c: // mov r/m16,  sreg16
            instr.mnemonic = mnem_mov;
            _cpu_decode_modrm(&addr, 0, 1, &instr.oper1, &instr.oper2);
            // convert reg to sreg
            instr.oper2 = REG_OPERAND(reg_to_sreg_tab[instr.oper2.reg / 2]);
            break;
        case 0x8e: // mov sreg16, r/m16
            instr.mnemonic = mnem_mov;
            _cpu_decode_modrm(&addr, 1, 1, &instr.oper1, &instr.oper2);
            instr.oper1 = REG_OPERAND(reg_to_sreg_tab[instr.oper1.reg / 2]);
            break;
        case 0x8d: // lea r16, m
            instr.mnemonic = mnem_lea;
            _cpu_decode_modrm(&addr, 1, 1, &instr.oper1, &instr.oper2);
            instr.oper2 = NO_OPERAND;
            break;
        case 0x8f: // pop r/m16
            instr.mnemonic = mnem_pop;
            _cpu_decode_modrm(&addr, 0, 1, &instr.oper1, &instr.oper2);
            instr.oper2 = NO_OPERAND;
            break;

        case 0x90: // nop
            instr.mnemonic = mnem_nop;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0x91: // xchg cx, ax
        case 0x92: // xchg dx, ax
        case 0x93: // xchg bx, ax
        case 0x94: // xchg sp, ax
        case 0x95: // xchg bp, ax
        case 0x96: // xchg si, ax
        case 0x97: // xchg di, ax
            instr.mnemonic = mnem_xchg;
            instr.oper1 = REG_OPERAND(xchg_tab[op - 0x91]);
            instr.oper2 = REG_OPERAND(reg_ax);
            break;
        case 0x98: // cbw
        case 0x99: // cwd
            instr.mnemonic = w ? mnem_cwd : mnem_cbw;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0x9a: // call segm:offs
            instr.mnemonic = mnem_call;
            instr.oper1 = MEM8_OPERAND(_cpu_decode_far(&addr));
            instr.oper2 = NO_OPERAND;
            break;
        case 0x9b: // wait
            instr.mnemonic = mnem_wait;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0x9c: // pushf
        case 0x9d: // popf
            instr.mnemonic = w ? mnem_popf : mnem_pushf;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0x9e: // sahf
        case 0x9f: // lahf
            instr.mnemonic = w ? mnem_lahf : mnem_sahf;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;

        case 0xa0: // mov al, m8
        case 0xa1: // mov ax, m16
        case 0xa2: // mov m8, al
        case 0xa3: // mov m16, ax
            instr.mnemonic = mnem_mov;
            instr.oper1 = REG_OPERAND(w ? reg_ax : reg_al);
            instr.oper2 = d
                ? MEM16_OPERAND(_cpu_decode_abs(&addr))
                : MEM8_OPERAND(_cpu_decode_abs(&addr));
            if(op & 2) {
                cpu_operand_t temp = instr.oper1;
                instr.oper1 = instr.oper2;
                instr.oper2 = temp;
            }
            break;
        case 0xa4: // movsb
        case 0xa5: // movsw
        case 0xa6: // cmpsb
        case 0xa7: // cmpsw
        case 0xaa: // stosb
        case 0xab: // stosw
        case 0xac: // lodsb
        case 0xad: // lodsw
        case 0xae: // scasb
        case 0xaf: // scasw
            if(op >= 0xaa) op -= 2;
            instr.mnemonic = str_tab[op - 0xa4];
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0xa8: // test al, imm8
        case 0xa9: // test ax, imm16
            instr.mnemonic = mnem_test;
            _cpu_decode_modrm(&addr, 1, w, &instr.oper1, &instr.oper2);
            instr.oper1 = REG_OPERAND(w ? reg_ax : reg_al);
            break;

        case 0xb0: // mov al, imm8
        case 0xb1: // mov cl, imm8
        case 0xb2: // mov dl, imm8
        case 0xb3: // mov bl, imm8
        case 0xb4: // mov ah, imm8
        case 0xb5: // mov ch, imm8
        case 0xb6: // mov dl, imm8
        case 0xb7: // mov bh, imm8
            instr.mnemonic = mnem_mov;
            instr.oper1 = REG_OPERAND(modrm_tab_reg_w0[op - 0xb0]);
            instr.oper2 = IMM8_OPERAND(READ(addr++));
            break;
        case 0xb8: // mov ax, imm16
        case 0xb9: // mov cx, imm16
        case 0xba: // mov dx, imm16
        case 0xbb: // mov bx, imm16
        case 0xbc: // mov sp, imm16
        case 0xbd: // mov bp, imm16
        case 0xbe: // mov si, imm16
        case 0xbf: // mov di, imm16
            instr.mnemonic = mnem_mov;
            instr.oper1 = REG_OPERAND(modrm_tab_reg_w1[op - 0xb8]);
            instr.oper2 = IMM16_OPERAND(_cpu_read16(&addr));
            break;

        case 0xc2: // ret imm8
            instr.mnemonic = mnem_ret;
            instr.oper1 = IMM8_OPERAND(READ(addr++));
            instr.oper2 = NO_OPERAND;
            break;
        case 0xc3: // ret
            instr.mnemonic = mnem_ret;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0xc4: // les reg16, mem32
        case 0xc5: // lds reg16, mem32
            instr.mnemonic = w ? mnem_lds : mnem_les;
            _cpu_decode_modrm(&addr, 1, 1, &instr.oper1, &instr.oper2);
            break;
        case 0xc6: // mov r/m8, imm8
        case 0xc7: // mov r/m16, imm16
            instr.mnemonic = mnem_mov;
            _cpu_decode_modrm(&addr, 0, w, &instr.oper1, &instr.oper2);
            break;
        case 0xca: // retf imm8
            instr.mnemonic = mnem_retf;
            instr.oper1 = IMM8_OPERAND(READ(addr++));
            instr.oper2 = NO_OPERAND;
            break;
        case 0xcb: // retf
            instr.mnemonic = mnem_retf;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0xcc: // int3
            instr.mnemonic = mnem_int;
            instr.oper1 = IMM8_OPERAND(3);
            instr.oper2 = NO_OPERAND;
            break;
        case 0xcd: // int imm8
            instr.mnemonic = mnem_int;
            instr.oper1 = IMM8_OPERAND(READ(addr++));
            instr.oper2 = NO_OPERAND;
            break;
        case 0xce: // into
            instr.mnemonic = mnem_into;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0xcf: // iret
            instr.mnemonic = mnem_iret;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        
        case 0xd0: // <alu> r/m8,  1
        case 0xd1: // <alu> r/m16, 1
            _cpu_decode_modrm(&addr, 0, w, &instr.oper1, &instr.oper2);
            instr.mnemonic = grp2_tab[instr.oper2.reg / 2];
            instr.oper2 = IMM8_OPERAND(1);
            if(instr.mnemonic == 0)
                instr.valid = 0;
            break;
        case 0xd2: // <alu> r/m8,  cl
        case 0xd3: // <alu> r/m16, cl
            _cpu_decode_modrm(&addr, 0, w, &instr.oper1, &instr.oper2);
            instr.mnemonic = grp2_tab[instr.oper2.reg / 2];
            instr.oper2 = REG_OPERAND(reg_cl);
            if(instr.mnemonic == 0)
                instr.valid = 0;
            break;
        case 0xd4: // aam imm8
        case 0xd5: // aad imm8
            instr.mnemonic = w ? mnem_aad : mnem_aam;
            instr.oper1 = IMM8_OPERAND(READ(addr++));
            instr.oper2 = NO_OPERAND;
            break;
        case 0xd7: // xlatb
            instr.mnemonic = mnem_xlatb;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0xe0: // loopnz
        case 0xe1: // loopz
        case 0xe2: // loop
        case 0xe3: // jcxz
            instr.mnemonic = loop_tab[op - 0xe0];
            instr.oper1 = IMM8_OPERAND(READ(addr++));
            instr.oper2 = NO_OPERAND;
            break;
        case 0xe4: // in al, imm8
        case 0xe5: // in ax, imm8
            instr.mnemonic = mnem_in;
            instr.oper1 = REG_OPERAND(w ? reg_ax : reg_al);
            instr.oper2 = IMM8_OPERAND(READ(addr++));
            break;
        case 0xe6: // out imm8, al
        case 0xe7: // out imm8, ax
            instr.mnemonic = mnem_out;
            instr.oper1 = IMM8_OPERAND(READ(addr++));
            instr.oper2 = REG_OPERAND(w ? reg_ax : reg_al);
            break;
        case 0xe8: // call rel16
        case 0xe9: // jmp  rel16
            instr.mnemonic = w ? mnem_jmp : mnem_call;
            instr.oper1 = IMM16_OPERAND(_cpu_read16(&addr));
            instr.oper2 = NO_OPERAND;
            break;
        case 0xea: // jmp segm:offs
            instr.mnemonic = mnem_jmp;
            instr.oper1 = MEM8_OPERAND(_cpu_decode_far(&addr));
            instr.oper2 = NO_OPERAND;
            break;
        case 0xeb: // jmp rel8
            instr.mnemonic = mnem_jmp;
            instr.oper1 = IMM8_OPERAND(READ(addr++));
            instr.oper2 = NO_OPERAND;
            break;
        case 0xec: // in al, dx
        case 0xed: // in ax, dx
            instr.mnemonic = mnem_in;
            instr.oper1 = REG_OPERAND(w ? reg_ax : reg_al);
            instr.oper2 = REG_OPERAND(reg_dx);
            break;
        case 0xee: // out dx, al
        case 0xef: // out dx, ax
            instr.mnemonic = mnem_out;
            instr.oper1 = REG_OPERAND(reg_dx);
            instr.oper2 = REG_OPERAND(w ? reg_ax : reg_al);
            break;

        case 0xf0: // lock
            ESP_LOGE(TAG, "unexpected LOCK prefix");
            instr.valid = 0;
            break;
        case 0xf2: // repnz
            ESP_LOGE(TAG, "unexpected REPNZ prefix");
            instr.valid = 0;
            break;
        case 0xf3: // repz
            ESP_LOGE(TAG, "unexpected REPZ prefix");
            instr.valid = 0;
            break;
        case 0xf4: // hlt
            instr.mnemonic = mnem_hlt;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0xf5: // cmc
            instr.mnemonic = mnem_cmc;
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0xf6: // <alu> r/m8,  <nothing>|imm8
        case 0xf7: // <alu> r/m16, <nothing>|imm8
            _cpu_decode_modrm(&addr, 0, w, &instr.oper1, &instr.oper2);
            instr.mnemonic = grp3_tab[instr.oper2.reg / 2];
            if(instr.mnemonic == mnem_test)
                instr.oper2 = IMM8_OPERAND(READ(addr++));
            else
                instr.oper2 = NO_OPERAND;
            break;
        case 0xf8: // clc
        case 0xf9: // stc
        case 0xfa: // cli
        case 0xfb: // sti
        case 0xfc: // cld
        case 0xfd: // std
            instr.mnemonic = flag_tab[op - 0xf8];
            instr.oper1 = NO_OPERAND;
            instr.oper2 = NO_OPERAND;
            break;
        case 0xfe: // inc|dec r/m8
            _cpu_decode_modrm(&addr, 0, 0, &instr.oper1, &instr.oper2);
            instr.mnemonic = instr.oper2.reg == 0 ? mnem_inc : mnem_dec;
            instr.oper2 = NO_OPERAND;
            break;
        case 0xff: // inc|dec r/m16; call [r/m16]; jmp r/m16; jmp [r/m16]; push r/m16
            _cpu_decode_modrm(&addr, 0, 0, &instr.oper1, &instr.oper2);
            switch(instr.oper2.reg) {
                case 0:
                    instr.mnemonic = mnem_inc;
                    break;
                case 1:
                    instr.mnemonic = mnem_dec;
                    break;
                case 3:
                    instr.oper1.type = operand_far_at_location;
                case 2:
                    instr.mnemonic = mnem_call;
                    break;
                case 5:
                    instr.oper1.type = operand_far_at_location;
                case 4:
                    instr.mnemonic = mnem_jmp;
                    break;
                default:
                    instr.mnemonic = mnem_push;
            }
            instr.oper2 = NO_OPERAND;
            break;

        default:
            instr.valid = 0;
    }

    instr.length = addr - addr_start;
    return instr;
}

void cpu_instr_sprint(cpu_instr_t instr, char* buf) {
    if(!instr.valid) {
        strcpy(buf, "invalid");
        return;
    }

    char buf1[32], buf2[32];
    _cpu_oper_sprint(instr.oper1, buf1, instr.so);
    _cpu_oper_sprint(instr.oper2, buf2, instr.so);

    // prefixes
    if(instr.rep)   buf += sprintf(buf, "rep ");
    if(instr.repne) buf += sprintf(buf, "repne ");

    if(strlen(buf1) == 0 && strlen(buf2) == 0) {
        strcpy(buf, instr_names[instr.mnemonic]);
    } else if(strlen(buf2) == 0) {
        sprintf(buf, "%s\t%s", instr_names[instr.mnemonic], buf1);
    } else {
        sprintf(buf, "%s\t%s, %s", instr_names[instr.mnemonic], buf1, buf2);
    }
}

void cpu_disasm(uint32_t addr, int32_t len) {
    char buf[32], buf2[32];
    while(len > 0) {
        cpu_instr_t instr = cpu_fetch_decode(addr);
        cpu_instr_sprint(instr, buf);
        _cpu_byte_sprint(addr, instr.length, buf2);
        ESP_LOGI("8086-disasm", "%05x: %18s %s", addr, buf2, buf);
        addr += instr.length;
        len -= instr.length;
    }
}

inline void _cpu_execute_jump(cpu_instr_t instr) {
    // see if we need to jump
    uint8_t condition_met;
    switch(instr.mnemonic) {
        case mnem_jmp:  condition_met = 1; break;
        case mnem_jo:   condition_met = READ_FLAG(FLAG_OF); break;
        case mnem_jno:  condition_met = !READ_FLAG(FLAG_OF); break;
        case mnem_jc:   condition_met = READ_FLAG(FLAG_CF); break;
        case mnem_jnc:  condition_met = !READ_FLAG(FLAG_CF); break;
        case mnem_jz:   condition_met = READ_FLAG(FLAG_ZF); break;
        case mnem_jnz:  condition_met = !READ_FLAG(FLAG_ZF); break;
        case mnem_jbe:  condition_met = READ_FLAG(FLAG_CF) || READ_FLAG(FLAG_ZF); break;
        case mnem_ja:   condition_met = !READ_FLAG(FLAG_CF) & !READ_FLAG(FLAG_ZF); break;
        case mnem_js:   condition_met = READ_FLAG(FLAG_SF); break;
        case mnem_jns:  condition_met = !READ_FLAG(FLAG_SF); break;
        case mnem_jp:   condition_met = READ_FLAG(FLAG_PF); break;
        case mnem_jnp:  condition_met = !READ_FLAG(FLAG_PF); break;
        case mnem_jl:   condition_met = READ_FLAG(FLAG_SF) != READ_FLAG(FLAG_OF); break;
        case mnem_jge:  condition_met = READ_FLAG(FLAG_SF) == READ_FLAG(FLAG_OF); break;
        case mnem_jle:  condition_met = (READ_FLAG(FLAG_SF) != READ_FLAG(FLAG_OF)) || READ_FLAG(FLAG_ZF); break;
        case mnem_jg:   condition_met = (READ_FLAG(FLAG_SF) == READ_FLAG(FLAG_OF)) && !READ_FLAG(FLAG_ZF); break;
        default:
            ESP_LOGE(TAG, "invalid jump instruction");
            return;
    }
    if(!condition_met) {
        regs.ip += instr.length;
        return;
    }
    // perform the jump
    if(instr.oper1.type == operand_imm8) // rel8
        regs.ip += instr.length + *(int8_t*)&instr.oper1.imm8;
    else if(instr.oper1.type == operand_imm16) // rel16
        regs.ip += instr.length + *(int16_t*)&instr.oper1.imm16;
    else if(instr.oper1.type == operand_mem8) { // segm16:offs16
        regs.cs = instr.oper1.mem.far_segm;
        regs.ip = instr.oper1.mem.far_offs;
    }
}
void cpu_run(void) {
    // fetch instruction
    cpu_instr_t instr = cpu_fetch_decode((uint32_t)regs.cs << 4 | regs.ip);

    #ifdef TRACE_VCPU
    char buf[32];
    cpu_instr_sprint(instr, buf);
    ESP_LOGI(TAG, "decoded[%04x:%04x]: %s", regs.cs, regs.ip, buf);
    #endif

    // macros
    #define RDOP_8(o)     _cpu_oprd8 ((o == 2) ? instr.oper2 : instr.oper1, instr.so)
    #define RDOP_16(o)    _cpu_oprd16((o == 2) ? instr.oper2 : instr.oper1, instr.so)
    #define WROP_8(o, v)  _cpu_opwr8 ((o == 2) ? instr.oper2 : instr.oper1, instr.so, v)
    #define WROP_16(o, v) _cpu_opwr16((o == 2) ? instr.oper2 : instr.oper1, instr.so, v)

    // determine bit size
    uint8_t op1w = instr.oper1.type == operand_imm16 || instr.oper1.type == operand_mem16;
    uint8_t op2w = instr.oper2.type == operand_imm16 || instr.oper2.type == operand_mem16;
    if(instr.oper2.type != operand_no && op1w != op2w)
        ESP_LOGE(TAG, "operand 1 width != operand 2 width");
    uint8_t w = op1w || op2w;

    // execute instruction
    // (another 500-line switch incoming!!!!!!)
    switch(instr.mnemonic) {
        case mnem_jmp:
        case mnem_jo:
        case mnem_jno:
        case mnem_jc:
        case mnem_jnc:
        case mnem_jz:
        case mnem_jnz:
        case mnem_jbe:
        case mnem_ja:
        case mnem_js:
        case mnem_jns:
        case mnem_jp:
        case mnem_jnp:
        case mnem_jl:
        case mnem_jge:
        case mnem_jle:
        case mnem_jg:
            _cpu_execute_jump(instr);
            return;

        case mnem_cli: WRITE_FLAG(FLAG_IF, 0); break;
        case mnem_sti: WRITE_FLAG(FLAG_IF, 1); break;
        case mnem_clc: WRITE_FLAG(FLAG_CF, 0); break;
        case mnem_stc: WRITE_FLAG(FLAG_CF, 1); break;
        case mnem_cmc: WRITE_FLAG(FLAG_CF, !READ_FLAG(FLAG_CF)); break;
        case mnem_cld: WRITE_FLAG(FLAG_DF, 0); break;
        case mnem_std: WRITE_FLAG(FLAG_DF, 1); break;

        case mnem_aaa:
            if((regs.al & 4) > 9 || READ_FLAG(FLAG_AF)) {
                regs.al += 6;
                regs.ah++;
                WRITE_FLAG(FLAG_AF, 1);
                WRITE_FLAG(FLAG_CF, 1);
            } else {
                WRITE_FLAG(FLAG_AF, 0);
                WRITE_FLAG(FLAG_CF, 0);
            }
            break;
        case mnem_aad:
            regs.al = (regs.ah * 10) + regs.al;
            regs.ah = 0;
            break;
        case mnem_aam:
            regs.ah = regs.al / 10;
            regs.al = regs.al % 10;
            break;
        case mnem_aas:
            if((regs.al & 4) > 9 || READ_FLAG(FLAG_AF)) {
                regs.al -= 6;
                regs.ah--;
                WRITE_FLAG(FLAG_AF, 1);
                WRITE_FLAG(FLAG_CF, 1);
            } else {
                WRITE_FLAG(FLAG_AF, 0);
                WRITE_FLAG(FLAG_CF, 0);
            }
            break;

        case mnem_adc:
            if(w) _cpu_set_fl(w, WROP_16(1, RDOP_16(1) + RDOP_16(2) + READ_FLAG(FLAG_CF)));
            else  _cpu_set_fl(w, WROP_8(1, RDOP_8(1) + RDOP_8(2) + READ_FLAG(FLAG_CF)));
            break;
        case mnem_add:
            if(w) _cpu_set_fl(w, WROP_16(1, RDOP_16(1) + RDOP_16(2)));
            else  _cpu_set_fl(w, WROP_8(1, RDOP_8(1) + RDOP_8(2)));
            break;
        case mnem_sbb:
            if(w) _cpu_set_fl(w, WROP_16(1, RDOP_16(1) - RDOP_16(2) - READ_FLAG(FLAG_CF)));
            else  _cpu_set_fl(w, WROP_8(1, RDOP_8(1) - RDOP_8(2) - READ_FLAG(FLAG_CF)));
            break;
        case mnem_sub:
            if(w) _cpu_set_fl(w, WROP_16(1, RDOP_16(1) - RDOP_16(2)));
            else  _cpu_set_fl(w, WROP_8(1, RDOP_8(1) - RDOP_8(2)));
            break;
        case mnem_and:
            if(w) _cpu_set_fl(w, WROP_16(1, RDOP_16(1) & RDOP_16(2)));
            else  _cpu_set_fl(w, WROP_8(1, RDOP_8(1) & RDOP_8(2)));
            WRITE_FLAG(FLAG_OF, 0);
            WRITE_FLAG(FLAG_CF, 0);
            break;
        case mnem_or:
            if(w) _cpu_set_fl(w, WROP_16(1, RDOP_16(1) | RDOP_16(2)));
            else  _cpu_set_fl(w, WROP_8(1, RDOP_8(1) | RDOP_8(2)));
            WRITE_FLAG(FLAG_OF, 0);
            WRITE_FLAG(FLAG_CF, 0);
            break;
        case mnem_xor:
            if(w) _cpu_set_fl(w, WROP_16(1, RDOP_16(1) ^ RDOP_16(2)));
            else  _cpu_set_fl(w, WROP_8(1, RDOP_8(1) ^ RDOP_8(2)));
            WRITE_FLAG(FLAG_OF, 0);
            WRITE_FLAG(FLAG_CF, 0);
            break;

        case mnem_lahf:
            regs.ah = regs.flags & 0xff;
            break;
        case mnem_sahf:
            regs.flags &= 0xff00;
            regs.flags |= regs.ah;
            break;

        case mnem_mov:
            if(w) WROP_16(1, RDOP_16(2));
            else  WROP_8(1, RDOP_8(2));
            break;

        default:
            ESP_LOGE(TAG, "%s is not implemented", instr_names[instr.mnemonic]);
    }

    #undef RDOP_8
    #undef RDOP_16
    #undef WROP_8
    #undef RDOP_16

    regs.ip += instr.length;
}