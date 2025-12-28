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

#ifndef __DEVICE_MAP_H__
#define __DEVICE_MAP_H__
//头文件保护宏：防止重复包含。
#include <cpu/difftest.h>
//依赖 difftest.h：NEMU 中的 difftest 用于和参考模型对比。
typedef void(*io_callback_t)(uint32_t, int, bool);
//定义了一个函数指针类型 io_callback_t，表示设备I/O读写回调函数uint32_t→地址 int→读写数据的长度 bool→读还是写
uint8_t* new_space(int size);
//在设备映射中，通常需要分配一段内存作为 设备寄存器空间 new_space 用来申请一段连续的模拟内存区域。
typedef struct {
  const char *name;
  // we treat ioaddr_t as paddr_t here
  paddr_t low;
  paddr_t high;
  void *space;
  io_callback_t callback;
} IOMap;
//表示一段I/O映射的设备信息:name:设备名字(调试用).low&high:物理地址范围.space:映射的内存空间(寄存器或存储).callback:访问该设备时的回调函数
static inline bool map_inside(IOMap *map, paddr_t addr) {
  return (addr >= map->low && addr <= map->high);
}//-> 用于通过 指针 访问结构体成员。. 用于通过 结构体实例 访问结构体成员。
//判断某个地址是否落在该设备映射范围内。
static inline int find_mapid_by_addr(IOMap *maps, int size, paddr_t addr) {
  int i;
  for (i = 0; i < size; i ++) {
    if (map_inside(maps + i, addr)) {
      difftest_skip_ref();
      return i;
    }
  }
  return -1;
}
//遍历所有 maps，找到地址属于哪一个设备。找到后调用 difftest_skip_ref()，表示 此访问不参与参考模型对比（因为设备访问不可预测）。
//返回设备 ID，没找到返回 -1

/*设备映射接口*/
void add_pio_map(const char *name, ioaddr_t addr,
        void *space, uint32_t len, io_callback_t callback);
void add_mmio_map(const char *name, paddr_t addr,
        void *space, uint32_t len, io_callback_t callback);
//add_pio_map：添加 端口 I/O 设备映射。
//add_mmio_map：添加 内存映射 I/O (MMIO) 设备映射。
//它们会把设备注册到 IOMap 表里

/*设备访问接口*/
word_t map_read(paddr_t addr, int len, IOMap *map);
void map_write(paddr_t addr, int len, word_t data, IOMap *map);
//map_read：从设备映射中读取数据。map_write：向设备写数据.它们会调用map->callback来处理设备的特殊逻辑（比如UART输出，VGA刷新等）
#endif
/*这段代码（map.h）本身只是 接口定义，它描述了一个流程：
设备注册 (add_pio_map/add_mmio_map)
地址匹配 (find_mapid_by_addr/map_inside)
设备访问 (map_read/map_write)
设备动作 (callback)
真正的执行顺序是 CPU 访存 → 判断是否命中设备 → 调用这些接口 → 触发设备逻辑。*/
