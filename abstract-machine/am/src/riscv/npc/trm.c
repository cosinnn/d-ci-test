#include <am.h>
#include <klib-macros.h>
#include "../riscv.h"
#include <stdio.h>

extern char _heap_start;
int main(const char *args);
#define UART_BASE 0x10000000
#define UART_RBR (UART_BASE + 0)
#define UART_THR (UART_BASE + 0)
#define UART_IER (UART_BASE + 1)
#define UART_IIR (UART_BASE + 2)
#define UART_FCR (UART_BASE + 2)
#define UART_LCR (UART_BASE + 3)
#define UART_MCR (UART_BASE + 4)
#define UART_LSR (UART_BASE + 5)
#define UART_MSR (UART_BASE + 6)
#define UART_SCR (UART_BASE + 7)
void uart_init() {
    outb(UART_LCR, 0x80);
    outb(UART_RBR, 0x36);
    outb(UART_IER, 0x00);
    outb(UART_LCR, 0x03);
    outb(UART_FCR, 0x07);
    outb(UART_MCR, 0x03);
    outb(UART_IER, 0x00);
}

extern char _pmem_start;
#define PMEM_SIZE (256 * 1024 * 1024)
#define PMEM_END  ((uintptr_t)&_pmem_start + PMEM_SIZE)

Area heap = RANGE(&_heap_start, PMEM_END);
static const char mainargs[MAINARGS_MAX_LEN] = TOSTRING(MAINARGS_PLACEHOLDER); // defined in CFLAGS

void putch(char ch) {
  while ((inb(UART_LSR) & 0x20) == 0);
  outb(UART_BASE, ch);
}

void halt(int code) {
  npc_trap(code);
  while (1);
}

void ysyx_init()
{
  uint32_t mvendorid,marchid;
  asm volatile ("csrr %0, 3857" : "=r"(mvendorid) :  : );
  asm volatile ("csrr %0, 3858" : "=r"(marchid) :  : );
  printf("mvendorid:0x%x\n",mvendorid);
  printf("marchid:%d\n",marchid);
}

void _trm_init() {
  uart_init();
  ysyx_init();
  int ret = main(mainargs);
  halt(ret);
}
