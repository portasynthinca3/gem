#include "cpu.h"
#include "machine.h"
#include <string.h>
#include <esp_log.h>
#include <stdlib.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define TAG "8086-exec"

// CPU state

static uint8_t running, halted, trace;
static cpu_regs_t regs;
static cpu_brkpnt_t breakpoints[CPU_MAX_BREAKPOINTS];
static SemaphoreHandle_t single_steps;
static SemaphoreHandle_t intr_mutex;
static SemaphoreHandle_t hlt_semaphore;

// Private functions

static inline uint32_t _cpu_sreg_base(cpu_segm_override_t so) {
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
static inline uint16_t _cpu_index(cpu_mem_mode_t mode) {
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
                case reg_cs: return regs.cs;
                case reg_ds: return regs.ds;
                case reg_ss: return regs.ss;
                case reg_es: return regs.es;
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
                case reg_cs: return regs.cs = val;
                case reg_ds: return regs.ds = val;
                case reg_ss: return regs.ss = val;
                case reg_es: return regs.es = val;
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
        case operand_mem16: {
            uint32_t addr = _cpu_effective_addr(op.mem, so);
            _cpu_write16(&addr, val);
            return val;
        }
        default:
            ESP_LOGE(TAG, "illegal operand type");
            return 0;
    }
}

static inline void _cpu_push(uint16_t val) {
    regs.sp -= 2;
    uint32_t addr = ((uint32_t)regs.ss << 4) + regs.sp;
    _cpu_write16(&addr, val);
}
static inline uint16_t _cpu_pop() {
    uint32_t addr = ((uint32_t)regs.ss << 4) + regs.sp;
    uint16_t val = _cpu_read16(&addr);
    regs.sp += 2;
    return val;
}

static inline uint8_t _cpu_parity(uint8_t w, uint32_t val) {
    uint8_t ones = 0;
    for(int i = 0; i < W_BITS(w); i++) {
        if(val & 1) ones++;
        val >>= 1;
    }
    return ones % 2;
}

static inline void _cpu_set_szp(uint8_t w, int32_t result) {
    WRITE_FLAG(FLAG_SF, result < 0);
    WRITE_FLAG(FLAG_ZF, result == 0);
    WRITE_FLAG(FLAG_PF, _cpu_parity(w, *(uint32_t*)&result));
}

static inline int16_t _cpu_sx(int8_t val) {
    if(val & 0x80)
        return 0xff00 | val;
    return val;
}

static inline void _cpu_execute_jump(cpu_instr_t instr) {
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
        case mnem_jcxz: condition_met = regs.cx == 0; break;
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
        regs.ip += instr.length + _cpu_sx(*(int8_t*)&instr.oper1.imm8);
    else if(instr.oper1.type == operand_imm16) // rel16
        regs.ip += instr.length + *(int16_t*)&instr.oper1.imm16;
    else if(instr.oper1.type == operand_mem8) { // segm16:offs16
        regs.cs = instr.oper1.mem.far_segm;
        regs.ip = instr.oper1.mem.far_offs;
    } else if(instr.oper1.type == operand_far_at_location) { // jumptable
        uint32_t addr = _cpu_effective_addr(instr.oper1.mem, instr.so);
        regs.cs = _cpu_read16(&addr);
        regs.ip = _cpu_read16(&addr);
    }
}

static inline uint8_t _cpu_is_oper_16bit(cpu_operand_t op) {
    return op.type == operand_imm16
        || op.type == operand_mem16
        || (op.type == operand_reg && op.reg % 2 == 0)
        || (op.type == operand_reg && op.reg >= reg_cs); // segment register
}

static inline uint32_t _cpu_rcl(uint32_t val, uint8_t amt, uint8_t w) {
    for(int i = 0; i < amt; i++) {
        uint8_t cf = val >> (W_BITS(w) - 1);
        val <<= 1;
        val |= READ_FLAG(FLAG_CF);
        WRITE_FLAG(FLAG_CF, cf);
    }
    if(amt == 1)
        WRITE_FLAG(FLAG_OF, !(val >> (W_BITS(w) - 1)) != !READ_FLAG(FLAG_CF));
    _cpu_set_szp(w, val);
    return val;
}

static inline uint32_t _cpu_rcr(uint32_t val, uint8_t amt, uint8_t w) {
    if(amt == 1)
        WRITE_FLAG(FLAG_OF, !(val >> (W_BITS(w) - 1)) != !READ_FLAG(FLAG_CF));
    for(int i = 0; i < amt; i++) {
        uint8_t cf = val & 1;
        val >>= 1;
        val |= READ_FLAG(FLAG_CF) << (W_BITS(w) - 1);
        WRITE_FLAG(FLAG_CF, cf);
    }
    _cpu_set_szp(w, val);
    return val;
}

static inline uint32_t _cpu_rol(uint32_t val, uint8_t amt, uint8_t w) {
    for(int i = 0; i < amt; i++) {
        uint8_t cf = val >> (W_BITS(w) - 1);
        val <<= 1;
        val |= cf;
    }
    if(amt != 0)
        WRITE_FLAG(FLAG_CF, val & 1);
    if(amt == 1)
        WRITE_FLAG(FLAG_OF, !(val >> (W_BITS(w) - 1)) != !READ_FLAG(FLAG_CF));
    _cpu_set_szp(w, val);
    return val;
}

static inline uint32_t _cpu_ror(uint32_t val, uint8_t amt, uint8_t w) {
    for(int i = 0; i < amt; i++) {
        uint8_t cf = val & 1;
        val >>= 1;
        val |= cf << (W_BITS(w) - 1);
    }
    if(amt != 0)
        WRITE_FLAG(FLAG_CF, val >> (W_BITS(w) - 1));
    if(amt == 1)
        WRITE_FLAG(FLAG_OF, !(val >> (W_BITS(w) - 1)) != !(val >> (W_BITS(w) - 2)));
    _cpu_set_szp(w, val);
    return val;
}

static inline uint32_t _cpu_shl(uint32_t val, uint8_t amt, uint8_t w) {
    for(int i = 0; i < amt; i++) {
        WRITE_FLAG(FLAG_CF, val >> (W_BITS(w) - 1));
        val <<= 1;
    }
    if(amt == 1)
        WRITE_FLAG(FLAG_OF, !(val >> (W_BITS(w) - 1)) != !READ_FLAG(FLAG_CF));
    _cpu_set_szp(w, val);
    return val;
}
static inline uint32_t _cpu_shr(uint32_t val, uint8_t amt, uint8_t w) {
    uint32_t orig_val = val;
    for(int i = 0; i < amt; i++) {
        WRITE_FLAG(FLAG_CF, val & 1);
        val >>= 1;
    }
    if(amt == 1)
        WRITE_FLAG(FLAG_OF, orig_val >> (W_BITS(w) - 1));
    _cpu_set_szp(w, val);
    return val;
}
static inline uint32_t _cpu_sar(uint32_t val, uint8_t amt, uint8_t w) {
    uint32_t sign = val & (1 << (W_BITS(w) - 1));
    for(int i = 0; i < amt; i++) {
        WRITE_FLAG(FLAG_CF, val & 1);
        val >>= 1;
        val |= sign;
    }
    if(amt == 1)
        WRITE_FLAG(FLAG_OF, 0);
    _cpu_set_szp(w, val);
    return val;
}

static inline uint32_t _cpu_sub(uint32_t a, uint32_t b, uint8_t w) {
    uint8_t c1 = a >> (W_BITS(w) - 1);
    uint8_t c2 = b >> (W_BITS(w) - 1);
    uint16_t result = a - b;
    _cpu_set_szp(w, result);
    uint8_t c3 = result >> (W_BITS(w) - 1);
    WRITE_FLAG(FLAG_OF, c1 != c2 && c3 != c1);
    return result;
}

static void _cpu_intr(uint8_t num) {
    _cpu_push(regs.flags);
    _cpu_push(regs.cs);
    _cpu_push(regs.ip);
    uint32_t addr = num * 4;
    regs.ip = _cpu_read16(&addr);
    regs.cs = _cpu_read16(&addr);
    WRITE_FLAG(FLAG_IF, 0);
}

// Public functions

void cpu_reset(void) {
    // reset registers
    memset(&regs, 0, sizeof(regs));
    regs.cs = 0xffff;

    // reset breakpoints
    memset(breakpoints, 0, sizeof(breakpoints));

    // reset flags
    #ifdef CPU_PAUSED_ON_STARTUP
    running = 0;
    #else
    running = 1;
    #endif
    trace = 0;
    halted = 0;

    // create semaphores
    single_steps = xSemaphoreCreateCounting(UINT32_MAX, 0);
    intr_mutex = xSemaphoreCreateMutex();
    hlt_semaphore = xSemaphoreCreateBinary();
}

void cpu_print_state(void) {
    ESP_LOGI(TAG, "cpu state: AX=%04x BX=%04x CX=%04x DX=%04x", regs.ax, regs.bx, regs.cx, regs.dx);
    ESP_LOGI(TAG, "cpu state: SI=%04x DI=%04x BP=%04x SP=%04x", regs.si, regs.di, regs.bp, regs.sp);
    ESP_LOGI(TAG, "cpu state: CS=%04x DS=%04x SS=%04x ES=%04x", regs.cs, regs.ds, regs.ss, regs.es);
    ESP_LOGI(TAG, "cpu state: IP=%04x FLAGS=%04x [%s%s%s%s%s%s%s%s%s]", regs.ip, regs.flags,
        READ_FLAG(FLAG_CF) ? "C" : "c",
        READ_FLAG(FLAG_PF) ? "P" : "p",
        READ_FLAG(FLAG_AF) ? "A" : "a",
        READ_FLAG(FLAG_ZF) ? "Z" : "z",
        READ_FLAG(FLAG_SF) ? "S" : "s",
        READ_FLAG(FLAG_TF) ? "T" : "t",
        READ_FLAG(FLAG_IF) ? "I" : "i",
        READ_FLAG(FLAG_DF) ? "D" : "d",
        READ_FLAG(FLAG_OF) ? "O" : "o");
}

void cpu_step(void) {
    // wait for an interrupt if halted
    if(halted && !xSemaphoreTake(hlt_semaphore, 0))
        return;
    halted = 0;

    xSemaphoreTake(intr_mutex, portMAX_DELAY);

    // fetch instruction
    cpu_instr_t instr = cpu_fetch_decode((uint32_t)regs.cs << 4 | regs.ip);

    // trigger breakpoints
    #if CPU_MAX_BREAKPOINTS != 0
    for(int i = 0; i < CPU_MAX_BREAKPOINTS; i++) {
        if(breakpoints[i].active && breakpoints[i].cs == regs.cs && breakpoints[i].ip == regs.ip) {
            running = 0;
            trace = 1;
            ESP_LOGI(TAG, "breakpoint %d hit", i);
            break;
        }
    }
    #endif

    if(trace) {
        char buf[32];
        cpu_instr_sprint(instr, buf);
        ESP_LOGI(TAG, "decoded instr at %04x:%04x: %s", regs.cs, regs.ip, buf);
    }

    // macros
    #define RDOP_8(o)     _cpu_oprd8 ((o == 2) ? instr.oper2 : instr.oper1, instr.so)
    #define RDOP_16(o)    _cpu_oprd16((o == 2) ? instr.oper2 : instr.oper1, instr.so)
    #define WROP_8(o, v)  _cpu_opwr8 ((o == 2) ? instr.oper2 : instr.oper1, instr.so, v)
    #define WROP_16(o, v) _cpu_opwr16((o == 2) ? instr.oper2 : instr.oper1, instr.so, v)
    #define RDOP(o)       (w ? RDOP_16(o)    : RDOP_8(o))
    #define WROP(o, v)    (w ? WROP_16(o, v) : WROP_8(o, v))

    // determine bit size
    uint8_t op1w = _cpu_is_oper_16bit(instr.oper1);
    uint8_t op2w = _cpu_is_oper_16bit(instr.oper2);
    if((instr.oper2.type != operand_no && op1w != op2w)
            && instr.mnemonic != mnem_in
            && instr.mnemonic != mnem_out)
        ESP_LOGW(TAG, "operand 1 width != operand 2 width");
    uint8_t w = op1w;

    // instruction should change there control vars
    uint8_t add_instr_length = 1;

    // execute instruction
    // (another 500-line switch incoming!!!!!!)
    switch(instr.mnemonic) {
        case mnem_jmp: case mnem_jcxz:
        case mnem_jo:  case mnem_jno:
        case mnem_jc:  case mnem_jnc:
        case mnem_jz:  case mnem_jnz:
        case mnem_jbe: case mnem_ja:
        case mnem_js:  case mnem_jns:
        case mnem_jp:  case mnem_jnp:
        case mnem_jl:  case mnem_jge:
        case mnem_jle: case mnem_jg:
            _cpu_execute_jump(instr);
            add_instr_length = 0;
            break;

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
            _cpu_set_szp(0, regs.al);
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
        case mnem_aam:
            regs.ah = regs.al / 10;
            regs.al = regs.al % 10;
            _cpu_set_szp(0, regs.al);
            break;
        case mnem_daa:
            if((regs.al & 4) > 9 || READ_FLAG(FLAG_AF)) {
                regs.al += 6;
                WRITE_FLAG(FLAG_AF, 1);
            }
            if(regs.al > 0x9f || READ_FLAG(FLAG_CF)) {
                regs.al += 0x60;
                WRITE_FLAG(FLAG_CF, 1);
            }
            _cpu_set_szp(0, regs.al);
            break;
        case mnem_das:
            if((regs.al & 4) > 9 || READ_FLAG(FLAG_AF)) {
                regs.al -= 6;
                WRITE_FLAG(FLAG_AF, 1);
            }
            if(regs.al > 0x9f || READ_FLAG(FLAG_CF)) {
                regs.al -= 0x60;
                WRITE_FLAG(FLAG_CF, 1);
            }
            _cpu_set_szp(0, regs.al);
            break;
        
        case mnem_adc:
        case mnem_add: {
            uint16_t result;
            uint8_t c1 = RDOP(1) >> (W_BITS(w) - 1);
            uint8_t c2 = RDOP(2) >> (W_BITS(w) - 1);
            uint8_t carry = (instr.mnemonic == mnem_adc) ? READ_FLAG(FLAG_CF) : 0;
            if(w) result = WROP_16(1, RDOP_16(1) + RDOP_16(2) + carry);
            else  result = WROP_8(1, RDOP_8(1) + RDOP_8(2) + carry);
            _cpu_set_szp(w, result);
            uint8_t c3 = result >> (W_BITS(w) - 1);
            WRITE_FLAG(FLAG_OF, c1 == c2 && c3 != c1);
            break;
        }
        case mnem_sbb:
            WROP(1, _cpu_sub(RDOP(1), RDOP(2) - READ_FLAG(FLAG_CF), w));
            break;
        case mnem_sub:
            WROP(1, _cpu_sub(RDOP(1), RDOP(2), w));
            break;
        case mnem_cmp: 
            _cpu_sub(RDOP(1), RDOP(2), w);
            break;
        case mnem_and:
            _cpu_set_szp(w, WROP(1, RDOP(1) & RDOP(2)));
            WRITE_FLAG(FLAG_OF, 0);
            WRITE_FLAG(FLAG_CF, 0);
            break;
        case mnem_test:
            _cpu_set_szp(w, RDOP(1) & RDOP(2));
            break;
        case mnem_or:
            _cpu_set_szp(w, WROP(1, RDOP(1) | RDOP(2)));
            WRITE_FLAG(FLAG_OF, 0);
            WRITE_FLAG(FLAG_CF, 0);
            break;
        case mnem_xor:
            _cpu_set_szp(w, WROP(1, RDOP(1) ^ RDOP(2)));
            WRITE_FLAG(FLAG_OF, 0);
            WRITE_FLAG(FLAG_CF, 0);
            break;
        case mnem_not:
            WROP(1, ~RDOP(1));
            break;
        case mnem_neg: {
            uint8_t c2 = RDOP(1) >> (W_BITS(w) - 1);
            uint16_t result = -RDOP(1);
            _cpu_set_szp(w, result);
            uint8_t c3 = result >> (W_BITS(w) - 1);
            WRITE_FLAG(FLAG_OF, 0 != c2 && c3 != 0);
            break;
        }

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

        case mnem_shr:
            WROP(1, _cpu_shr(RDOP(1), RDOP_8(2), w));
            break;
        case mnem_sar:
            WROP(1, _cpu_sar(RDOP(1), RDOP_8(2), w));
            break;
        case mnem_shl:
            WROP(1, _cpu_shl(RDOP(1), RDOP_8(2), w));
            break;
        case mnem_rol:
            WROP(1, _cpu_rol(RDOP(1), RDOP_8(2), w));
            break;
        case mnem_rcl:
            WROP(1, _cpu_rcl(RDOP(1), RDOP_8(2), w));
            break;
        case mnem_ror:
            WROP(1, _cpu_ror(RDOP(1), RDOP_8(2), w));
            break;
        case mnem_rcr:
            WROP(1, _cpu_rcr(RDOP(1), RDOP_8(2), w));
            break;

        case mnem_cbw:
            regs.ah = (regs.al & 0x80) ? 255 : 0;
            break;
        case mnem_cwd:
            regs.dx = (regs.ax & 0x8000) ? 0xffff : 0;
            break;

        case mnem_movsb:
        case mnem_movsw:
            do {
                uint32_t ds_si = ((uint32_t)regs.ds << 4) + regs.si;
                uint32_t es_di = ((uint32_t)regs.es << 4) + regs.di;
                if(w) _cpu_write16(&es_di, _cpu_read_16(&ds_si));
                else  WRITE(es_di, READ(ds_si));
                if(READ_FLAG(FLAG_DF)) {
                    regs.si -= w + 1;
                    regs.di -= w + 1;
                } else {
                    regs.si += w + 1;
                    regs.di += w + 1;
                }
            // stopping conditions:
            // 1. instruction isn't prefixed by REP
            // 2. --CX is zero if prefixed by either REP
            } while(instr.rp == rp_no || --regs.cx);
            break;
        case mnem_lodsb:
        case mnem_lodsw:
            do {
                uint32_t ds_si = ((uint32_t)regs.ds << 4) + regs.si;
                if(w) regs.ax = _cpu_read16(ds_si);
                else  regs.al = READ(ds_si);
                if(READ_FLAG(FLAG_DF))
                    regs.si -= w + 1;
                else
                    regs.si += w + 1;
            // stopping conditions:
            // 1. instruction isn't prefixed by REP
            // 2. --CX is zero if prefixed by either REP
            } while(instr.rp == rp_no || --regs.cx);
            break;
        case mnem_stosb:
        case mnem_stosw:
            do {
                uint32_t es_di = ((uint32_t)regs.es << 4) + regs.di;
                if(w) _cpu_write16(es_di, regs.ax);
                else  WRITE(es_di, regs.al);
                if(READ_FLAG(FLAG_DF))
                    regs.si -= w + 1;
                else
                    regs.si += w + 1;
            // stopping conditions:
            // 1. instruction isn't prefixed by REP
            // 2. --CX is zero if prefixed by either REP
            } while(instr.rp == rp_no || --regs.cx);
            break;
        case mnem_cmpsb:
        case mnem_cmpsw:
            do {
                uint32_t ds_si = ((uint32_t)regs.ds << 4) + regs.si;
                uint32_t es_di = ((uint32_t)regs.es << 4) + regs.di;
                if(w) _cpu_sub(_cpu_read16(&ds_si), _cpu_read16(&es_di), w);
                else  _cpu_sub(READ(ds_si), READ(es_di), w);
                if(READ_FLAG(FLAG_DF)) {
                    regs.si -= w + 1;
                    regs.di -= w + 1;
                } else {
                    regs.si += w + 1;
                    regs.di += w + 1;
                }
            // stopping conditions:
            // 1. instruction is prefixed neither by REP/REPE nor REPNE (single-shot)
            // 2. --CX is zero if prefixed by either REP/REPE or REPNE
            // 3. ZF = 0 if prefixed by REP/REPE
            // 4. ZF = 1 if prefixed by REPNE
            } while(instr.rp == rp_no || (--regs.cx && (!READ_FLAG(FLAG_ZF) == !(instr.rp == rp_rep))));
            break;
        case mnem_scasb:
        case mnem_scasw:
            do {
                uint32_t ds_si = ((uint32_t)regs.ds << 4) + regs.si;
                if(w) _cpu_sub(regs.ax, _cpu_read16(ds_si), w);
                else  _cpu_sub(regs.al, READ(ds_si), w);
                if(READ_FLAG(FLAG_DF))
                    regs.si -= w + 1;
                else
                    regs.si += w + 1;
            // stopping conditions:
            // 1. instruction is prefixed neither by REP/REPE nor REPNE (single-shot)
            // 2. --CX is zero if prefixed by either REP/REPE or REPNE
            // 3. ZF = 0 if prefixed by REP/REPE
            // 4. ZF = 1 if prefixed by REPNE
            } while(instr.rp == rp_no || (--regs.cx && (!READ_FLAG(FLAG_ZF) == !(instr.rp == rp_rep))));
            break;

        case mnem_call:
            if(instr.oper1.type == operand_imm16) { // rel16
                _cpu_push(regs.ip + instr.length);
                regs.ip += instr.length + *(int16_t*)&instr.oper1.imm16;
            } else if(instr.oper1.type == operand_mem8) { // segm16:offs16
                _cpu_push(regs.cs);
                _cpu_push(regs.ip + instr.length);
                regs.cs = instr.oper1.mem.far_segm;
                regs.ip = instr.oper1.mem.far_offs;
            } else if(instr.oper1.type == operand_far_at_location) { // jumptable (calltable?)
                _cpu_push(regs.cs);
                _cpu_push(regs.ip + instr.length);
                uint32_t addr = _cpu_effective_addr(instr.oper1.mem, instr.so);
                regs.cs = _cpu_read16(&addr);
                regs.ip = _cpu_read16(&addr);
            }
            add_instr_length = 0;
            break;
        case mnem_ret:
            regs.ip = _cpu_pop();
            if(instr.oper1.type == operand_imm16)
                regs.sp += instr.oper1.imm16;
            add_instr_length = 0;
            break;
        case mnem_retf:
            regs.ip = _cpu_pop();
            if(instr.oper1.type == operand_imm16)
                regs.sp += instr.oper1.imm16;
            regs.cs = _cpu_pop();
            add_instr_length = 0;
            break;

        case mnem_inc: {
            uint8_t c1 = RDOP(1) >> (W_BITS(w) - 1);
            uint16_t result = WROP(1, RDOP(1) + 1);
            _cpu_set_szp(w, result);
            uint8_t c3 = result >> (W_BITS(w) - 1);
            WRITE_FLAG(FLAG_OF, c1 == 0 && c3 != c1);
            break;
        }
        case mnem_dec:
            WROP(1, _cpu_sub(RDOP(1), 1, w));
            break;

        case mnem_in:
            if(w) WROP(1, machine_io_rd16(RDOP_16(2)));
            else  WROP(1, machine_io_rd8(RDOP_16(2)));
            break;
        case mnem_out:
            if(op2w) machine_io_wr16(RDOP_16(1), RDOP_16(2));
            else     machine_io_wr8(RDOP_16(1), RDOP_8(2));
            break;

        case mnem_loop:
        case mnem_loopz:
        case mnem_loopnz: {
            regs.cx--;
            uint8_t should_jump = regs.cx != 0;
            if(instr.mnemonic == mnem_loopz)
                should_jump = should_jump && READ_FLAG(FLAG_ZF); // &&= is not a thing :(
            else if(instr.mnemonic == mnem_loopnz)
                should_jump = should_jump && !READ_FLAG(FLAG_ZF);
            if(should_jump) {
                regs.ip += instr.length + _cpu_sx(*(int8_t*)&instr.oper1.imm8);
                add_instr_length = 0;
            }
            break;
        }

        case mnem_lds:
        case mnem_les:
            WROP_16(1, RDOP_16(2));
            instr.oper2.mem.disp8 += 2; // HAX!
            instr.oper2.mem.disp16 += 2;
            if(instr.mnemonic == mnem_lds)
                regs.ds = RDOP_16(2);
            else
                regs.es = RDOP_16(2);
            break;
        
        case mnem_lea:
            WROP_16(1, _cpu_effective_addr(instr.oper2.mem, instr.so));
            break;

        case mnem_nop:
        case mnem_wait: // we don't have an fpu
            break;

        case mnem_push:
            _cpu_push(RDOP_16(1));
            break;
        case mnem_pushf:
            _cpu_push(regs.flags);
            break;
        case mnem_pop:
            WROP_16(1, _cpu_pop());
            break;
        case mnem_popf:
            regs.flags = _cpu_pop();
            break;

        case mnem_xchg:
            if(w) {
                uint16_t tmp = RDOP_16(1);
                WROP_16(1, RDOP_16(2));
                WROP_16(2, tmp);
            } else {
                uint8_t tmp = RDOP_8(1);
                WROP_8(1, RDOP_8(2));
                WROP_8(2, tmp);
            }
            break;

        case mnem_xlatb:
            regs.al = ((uint32_t)regs.ds << 4) + regs.bx + regs.al;
            break;

        case mnem_hlt:
            halted = 1;
            break;
        case mnem_iret:
            add_instr_length = 0;
            regs.ip = _cpu_pop();
            regs.cs = _cpu_pop();
            regs.flags = _cpu_pop();
            break;
        case mnem_int:
            regs.ip += instr.length;
            add_instr_length = 0;
            _cpu_intr(RDOP_8(1));
            break;
        case mnem_into:
            if(READ_FLAG(FLAG_OF)) {
                regs.ip += instr.length;
                add_instr_length = 0;
                _cpu_intr(4);
            }
            break;
    }

    #undef RDOP_8
    #undef RDOP_16
    #undef WROP_8
    #undef RDOP_16

    // print state after instruction
    if(trace)
        cpu_print_state();

    if(add_instr_length)
        regs.ip += instr.length;

    xSemaphoreGive(intr_mutex);
}

void cpu_nmi(void) {
    // wait for CPU to finish current cycle
    xSemaphoreTake(intr_mutex, portMAX_DELAY);
    if(halted)
        xSemaphoreGive(hlt_semaphore);
    _cpu_intr(2);
    xSemaphoreGive(intr_mutex);
}

void cpu_intr(uint8_t num) {
    xSemaphoreTake(intr_mutex, portMAX_DELAY);
    if(!READ_FLAG(FLAG_IF))
        return;
    if(halted)
        xSemaphoreGive(hlt_semaphore);
    _cpu_intr(num);
    xSemaphoreGive(intr_mutex);
}

void cpu_loop(void) {
    while(1) {
        if(running || xSemaphoreTake(single_steps, 0))
            cpu_step();
    }
}

// Debugging functions

void cpu_set_running(uint8_t val) {
    ESP_LOGI(TAG, "%s", val ? "running" : "stopped");
    running = val;
}

uint8_t cpu_running(void) {
    return running;
}

void cpu_single_step(uint32_t steps) {
    for(uint32_t i = 0; i < steps; i++)
        xSemaphoreGive(single_steps);
}

void cpu_set_trace(uint8_t val) {
    ESP_LOGI(TAG, "tracing %s", val ? "enabled" : "disabled");
    trace = val;
}

uint8_t cpu_trace(void) {
    return trace;
}

uint8_t cpu_breakpoint(uint16_t cs, uint16_t ip) {
    // find unused slot
    for(int i = 0; i < CPU_MAX_BREAKPOINTS; i++) {
        if(!breakpoints[i].active) {
            breakpoints[i].cs = cs;
            breakpoints[i].ip = ip;
            breakpoints[i].active = 1;
            ESP_LOGI(TAG, "inserted breakpoint at %04x:%04x (id %d)", cs, ip, i);
            return i;
        }
    }
    ESP_LOGE(TAG, "no breakpoint slots left");
    return -1;
}

void cpu_del_breakpoint(uint8_t num) {
    breakpoints[num].active = 0;
    ESP_LOGI(TAG, "breakpoint remved (id %d)", num);
}

void cpu_list_breakpoints(void) {
    for(int i = 0; i < CPU_MAX_BREAKPOINTS; i++)
        if(breakpoints[i].active)
            ESP_LOGI(TAG, "breakpoint %d at %04x:%04x", i, breakpoints[i].cs, breakpoints[i].ip);
}

cpu_regs_t cpu_state(void) {
    return regs;
}