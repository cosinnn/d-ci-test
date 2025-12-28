#include <am.h>
#include <klib-macros.h>
#include <stdio.h>
#include <riscv/riscv.h>

extern char _heap_start;
int main(const char *args);

extern char _pmem_start;
#define PMEM_SIZE (128 * 1024 * 1024)
#define PMEM_END  ((uintptr_t)&_pmem_start + PMEM_SIZE)
#define SERIAL_PORT 0xa0000038//0x10000000
Area heap = RANGE(&_heap_start, PMEM_END);
//static const char mainargs[MAINARGS_MAX_LEN] = TOSTRING(MAINARGS_PLACEHOLDER); // defined in CFLAGS
static const char mainargs[MAINARGS_MAX_LEN] = TOSTRING(MAINARGS);
#define UART_BASE 0x10000000
#define THR_ADDR (UART_BASE + 0x0)
#define LER_ADDR (UART_BASE + 0x1)
#define IIR_ADDR (UART_BASE + 0x2)
#define FCR_ADDR (UART_BASE + 0x2)
#define LCR_ADDR (UART_BASE + 0x3)
#define MCR_ADDR (UART_BASE + 0x4)
#define LSR_ADDR (UART_BASE + 0x5)
#define MSR_ADDR (UART_BASE + 0x6)
#define LSB_ADDR (UART_BASE + 0x0)
#define MSB_ADDR (UART_BASE + 0x1)

void putch(char ch) {
  //*(volatile char *)SERIAL_PORT = ch;
    while ((inb(LSR_ADDR) & 0x20) == 0) {
    }
  outb(THR_ADDR, ch);
}

void halt(int code) {
  asm volatile("mv a0, %0; ebreak" : :"r"(code));
  while (1);
}

void _trm_init() {
    // unsigned int vendor_id, arch_id;
    // asm volatile("csrrw %0, mvendorid, x0" : "=r"(vendor_id));
    // asm volatile("csrrw %0, marchid, x0"   : "=r"(arch_id));

    // printf("mvendorid = %d (0x%x)\n", vendor_id, vendor_id);
    // printf("marchid   = %d (0x%x)\n", arch_id, arch_id);
       // 配置除数寄存器
    outb(LCR_ADDR, 0x80); 
    outb(LSB_ADDR, 0x36); 
    outb(MSB_ADDR, 0x00); 
    outb(LCR_ADDR, 0x03); 
    outb(FCR_ADDR, 0x07);
    outb(MCR_ADDR, 0x03); 
    outb(LER_ADDR, 0x00);
    //putch(mainargs[0]);
  int ret = main(mainargs);
  halt(ret);
}
