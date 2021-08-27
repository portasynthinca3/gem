#include "cpu.h"
#include "machine.h"
#include <string.h>
#include <esp_log.h>
#include <stdlib.h>
#include <stdio.h>

#define TAG "8086-disasm"

// Names

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
    "retf",  "rol",   "ror",   "sahf",  "not",  "sar",   "sbb",
    "scasb", "scasw", "shl",   "shr",   "stc",  "std",   "sti",
    "stosb", "stosw", "sub",   "test",  "xchg", "xlatb", "xor",
    "wait",  "jcxz",  "jmp",   "hlt"
};

// Private functions

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

// Public functions

void cpu_instr_sprint(cpu_instr_t instr, char* buf) {
    if(!instr.valid) {
        strcpy(buf, "invalid");
        return;
    }

    char buf1[32], buf2[32];
    _cpu_oper_sprint(instr.oper1, buf1, instr.so);
    _cpu_oper_sprint(instr.oper2, buf2, instr.so);

    // prefixes
    if(instr.rp == rp_rep)   buf += sprintf(buf, "rep ");
    if(instr.rp == rp_repne) buf += sprintf(buf, "repne ");

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
        ESP_LOGI(TAG, "%05x: %18s %s", addr, buf2, buf);
        addr += instr.length;
        len -= instr.length;
    }
}