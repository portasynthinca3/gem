#include "pic.h"
#include "cpu.h"
#include "machine.h"
#include <string.h>
#include <esp_log.h>
#include <stdlib.h>
#include <stdio.h>

#define TAG "pic"

static uint8_t vector_offs;
static uint8_t imr, irr, isr;
static pic_state_t state;

void pic_irq(uint8_t num) {
    if(!(imr & (1 << num))) // check mask
        irr |= (1 << num);
}

void pic_clock(void) {
    // try to issue interrupts
    for(int i = 0; i < 8; i++) {
        if(irr & (1 << i) && cpu_intr(i + vector_offs)) {
            isr |= 1 << i;
            irr &= ~(1 << i);
            return;
        }
    }
}

void pic_port_wr8(uint8_t port, uint8_t data) {
    if(port == PIC_CMD_PORT) {
        if(data & 0x10) { // initialization command
            imr = 0;
            state = awaiting_icw2;
        }
    }
}

uint8_t pic_port_rd8(uint8_t port) {
    return 0;
}