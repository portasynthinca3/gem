#pragma once

#include <stdint.h>

// Definitions

// #define TRACE_MEM_ACCESS
#define TRACE_IO_ACCESS

#define BIOS_START 0xfe000
#define RAM_SIZE   64 * 1024

#define READ(addr)       machine_mem_rd((addr))
#define WRITE(addr, val) machine_mem_wr((addr), (val))

// Private functions

void     _machine_mem_wr (uint32_t addr, uint8_t val);
uint8_t  _machine_mem_rd (uint32_t addr);

void     _machine_io_wr8 (uint16_t port, uint8_t val);
void     _machine_io_wr16(uint16_t port, uint16_t val);
uint8_t  _machine_io_rd8 (uint16_t port);
uint16_t _machine_io_rd16(uint16_t port);

// Public functions

void    machine_init  (void);

#ifdef TRACE_MEM_ACCESS
void    machine_mem_wr(uint32_t addr, uint8_t val);
uint8_t machine_mem_rd(uint32_t addr);
#else
#define machine_mem_wr _machine_mem_wr
#define machine_mem_rd _machine_mem_rd
#endif

#ifdef TRACE_IO_ACCESS
void     machine_io_wr8 (uint16_t port, uint8_t val);
void     machine_io_wr16(uint16_t port, uint16_t val);
uint8_t  machine_io_rd8 (uint16_t port);
uint16_t machine_io_rd16(uint16_t port);
#else
#define machine_io_wr8  _machine_io_wr8
#define machine_io_wr16 _machine_io_wr16
#define machine_io_rd8  _machine_io_rd8
#define machine_io_rd16 _machine_io_rd16
#endif