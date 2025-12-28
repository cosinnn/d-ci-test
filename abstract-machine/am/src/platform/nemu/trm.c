#include <am.h>
#include <nemu.h>

extern char _heap_start;
int main(const char *args);

Area heap = RANGE(&_heap_start, PMEM_END);
static const char mainargs[MAINARGS_MAX_LEN] = TOSTRING(MAINARGS_PLACEHOLDER); // defined in CFLAGS将参书转化为字符串字面量

void putch(char ch) {
  outb(SERIAL_PORT, ch);//向指定的I/O端口写入一个字节的数据  
}

void halt(int code) {
  nemu_trap(code);

  // should not reach here
  while (1);
}

void _trm_init() {//程序的入口点，负责初始化运行时环境并启动程序
  int ret = main(mainargs);//0表示程序执行成功的退出码，非0值表示程序错误或异常
  halt(ret);
}
