/***************************************************************************************
* Copyright (c) 2014-2024 Zihao Yu, Nanjing University
*
* NEMU is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

#include <isa.h>
#include <memory/host.h>
#include <memory/vaddr.h>
#include <device/map.h>
// isa.h：和 CPU 架构相关的东西（寄存器等）。
// host.h：宿主机的内存操作函数（host_read / host_write）。
// vaddr.h：虚拟地址相关。

#define IO_SPACE_MAX (32 * 1024 * 1024)

static uint8_t *io_space = NULL;
static uint8_t *p_space = NULL;

/*定义最大 I/O 空间：32MB。
io_space：I/O 模拟内存的起始地址。
p_space：当前可用位置的指针（分配设备内存时往后推）*/

uint8_t* new_space(int size) {
  uint8_t *p = p_space;
  // page aligned;
  size = (size + (PAGE_SIZE - 1)) & ~PAGE_MASK;
  p_space += size;
  assert(p_space - io_space < IO_SPACE_MAX);
  return p;
}
/*new_space(size) 用来给设备分配一块内存空间（模拟寄存器）。
分配时会对齐到页大小（PAGE_SIZE），保证地址对齐。
更新 p_space 指针，相当于“堆分配”。
返回分配好的起始地址。*/

static void check_bound(IOMap *map, paddr_t addr) {
  if (map == NULL) {
    Assert(map != NULL, "address (" FMT_PADDR ") is out of bound at pc = " FMT_WORD, addr, cpu.pc);
  } else {
    Assert(addr <= map->high && addr >= map->low,
        "address (" FMT_PADDR ") is out of bound {%s} [" FMT_PADDR ", " FMT_PADDR "] at pc = " FMT_WORD,
        addr, map->name, map->low, map->high, cpu.pc);
  }
}
/*确认 addr 在设备的映射区间 [low, high] 里。
如果 map == NULL 或地址不在范围内，就报错,防止 CPU 访问了没注册的设备地址*/

static void invoke_callback(io_callback_t c, paddr_t offset, int len, bool is_write) {
  if (c != NULL) { c(offset, len, is_write); }
}
/*如果设备注册了 callback，调用它。
参数：
offset：设备内的偏移量（相对于 map->low）。
len：访问的字节数。
is_write：true 表示写，false 表示读。
不同设备的回调逻辑不同，比如 UART 会打印字符，VGA 会刷新。*/

void dtrace_read(const char *operation, paddr_t addr, int len, IOMap *map) {
    log_write("[%s] %s access at 0x%x, len = %d\n", 
           map->name, operation, addr, len);
}
void dtrace_write(const char *operation, paddr_t addr, int len, IOMap *map) {
    log_write("[%s] %s access at 0x%x, len = %d\n", 
           map->name, operation, addr, len);
}
void init_map() {
  io_space = malloc(IO_SPACE_MAX);
  assert(io_space);
  p_space = io_space;
}

word_t map_read(paddr_t addr, int len, IOMap *map) {
  assert(len >= 1 && len <= 8);
  check_bound(map, addr);
  paddr_t offset = addr - map->low;
  dtrace_read("READ", addr, len, map);
  invoke_callback(map->callback, offset, len, false); // prepare data to read
  word_t ret = host_read(map->space + offset, len);
  return ret;
}
/*读流程总结（用 UART 举例）
CPU 执行：lw a0, 0x10000000
调用 map_read(0x10000000, 4, uart_map)。
assert 检查 len=4 合法。
check_bound 确认 0x10000000 在 UART 地址范围 [0x10000000, 0x10000007] 内。
计算偏移：offset = 0。
invoke_callback 调用 UART 的回调（比如先刷新输入缓冲区）。
host_read(uart_space + 0, 4) → 从 UART 的数据寄存器取出值。
返回这个值 → 最终写进寄存器 a0。*/

void map_write(paddr_t addr, int len, word_t data, IOMap *map) {
  assert(len >= 1 && len <= 8);
  check_bound(map, addr);
  paddr_t offset = addr - map->low;
  dtrace_write("WRITE", addr, len, map);
  host_write(map->space + offset, len, data);
  invoke_callback(map->callback, offset, len, true);
}
