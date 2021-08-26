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