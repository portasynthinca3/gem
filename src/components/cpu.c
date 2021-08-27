// Resources I used:
// General encoding description: http://aturing.umcs.maine.edu/~meadow/courses/cos335/8086-instformat.pdf
// Opcode map: http://mlsite.net/8086/
// Operation: https://dsearls.org/courses/C391OrgSys/IntelAL/8086_instruction_set.html#JC

// Okay, lets talk about the architecture of this code for a bit.
// The 8086 emulator is split into four files:
//  - cpu_fetch.c
//    It is doing all the fetching and decoding work. Its main point of interest is the cpu_fetch_decode
//    function. It fetches an instruction and represents it using internal structs much like a human would
//    read it. Each instruction has three parts: the mnemonic, the destination operand (operand 1) and the
//    source operand (operand 2). What's the difference between it and the machine code it "decoded" then?
//    Well. Let's take the instruction `cmp al, 123` as an example. It can be encoded in two ways: using the
//    3C opcode which is a special opcode for comparing AL to an immediate, or using the general register-
//    -or-memory-to-immediate comparison opcode 80 /7. The latter uses an additional byte and probably takes
//    more time to execute on a real processor too, and that's probably why they made that "compare AL to
//    immediate" instruction. Both of them should show up in the disassembly as `cmp al, 123` though.
//    So, this decoder function essentially represents the instruction as a sort of tree:
//    - instruction: compare
//      - operand 1: present
//        - type: register
//        - register: AL
//      - operand 2: present
//        - type: 8-bit immediate
//        - immediate value: 123
//    Yes, this way of doing things is probably slower than if I combined all three (fetch, decode and execute)
//    cycles together in one function, but it's way more readable, facilitates expandability (what if I decide to 
//    emulate the 386 at some point?) and allows the disassembly code to not have its own decoder.
//  - cpu_exec.c
//    Executes what the decoder has represented. Easy, right?
//  - cpu_disasm.c
//    Prints what the decoder has represented. Very easy, right?
//  - cpu_repl.c
//    Provides the UART REPL to debug the CPU or the program running on it

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

void _cpu_write16(uint32_t* addr, uint16_t val) {
    WRITE((*addr)++, val & 0xff);
    WRITE((*addr)++, val >> 8);
}