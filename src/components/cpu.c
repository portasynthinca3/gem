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
    "jle",   "jg",    "lahf",  "lds",   "lea",
    "les",   "lodsb", "lodsw", "loop",  "mov",  "movsb", "movsw",
    "mul",   "neg",   "nop",   "or",    "out",  "pop",   "popa",
    "popf",  "push",  "pusha", "pushf", "rcl",  "rcr",   "ret",
    "retf",  "rol",   "ror",   "sahf",  "sal",  "sar",   "sbb",
    "scasb", "scasw", "shl",   "shr",   "stc",  "std",   "sti",
    "stosb", "stosw", "sub",   "test",  "xchg", "xlatb", "xor",
    "wait"
};

// Private functions

uint16_t _cpu_read16(uint32_t* addr) {
    uint16_t val = READ((*addr)++);
    val |= (uint16_t)READ((*addr)++) << 8;
    return val;
}

cpu_mem_oper_t _cpu_decode_far(uint32_t* addr) {
    uint16_t offs = _cpu_read16(addr);
    uint16_t segm = _cpu_read16(addr);

    return (cpu_mem_oper_t){.mode = disp_far, .far_offs = offs, .far_segm = segm};
}

cpu_mem_oper_t _cpu_decode_abs(uint32_t* addr) {
    uint16_t offs = _cpu_read16(addr);

    return (cpu_mem_oper_t){.mode = disp_abs, .disp16 = offs};
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
            op2->reg = (w ? modrm_tab_reg_w1 : modrm_tab_reg_w0)[reg];
    }

    // swap operands around (r/m <- reg instead of reg <- r/m)
    if(!d) {
        cpu_operand_t temp = *op1;
        *op1 = *op2;
        *op2 = temp;
    }
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
            instr.oper1 = REG_OPERAND(modrm_tab_reg_w1[op - 0xb0]);
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

void cpu_disasm(uint32_t addr, uint32_t len) {
    char buf[64], buf2[32];
    while(len) {
        cpu_instr_t instr = cpu_fetch_decode(addr);
        cpu_instr_sprint(instr, buf);
        _cpu_byte_sprint(addr, instr.length, buf2);
        ESP_LOGI("8086-disasm", "%06x: %18s %s", addr, buf2, buf);
        addr += instr.length;
        len -= instr.length;
    }
}

void cpu_run(void) {

}