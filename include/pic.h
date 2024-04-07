#pragma once

#include <stdint.h>

// Definitions

#define PIC_CMD_PORT 0x20
#define PIC_DAT_PORT 0x21

#define PIC_CMD_INIT 0x11

typedef enum {
    normal, awaiting_icw2, awaiting_icw3, awaiting_icw4
} pic_state_t;

// Public functions

void pic_irq  (uint8_t num);
void pic_clock(void);

// these are only supposed to be called by machine.c
void    pic_port_wr8(uint8_t port, uint8_t data);
uint8_t pic_port_rd8(uint8_t port);