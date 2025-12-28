#include <am.h>
#include <klib.h>
#include <klib-macros.h>

#if !defined(__ISA_NATIVE__) || defined(__NATIVE_USE_KLIB__)
static unsigned long int next = 1;

int rand(void) {
  // RAND_MAX assumed to be 32767
  next = next * 1103515245 + 12345;
  return (unsigned int)(next/65536) % 32768;
}

void srand(unsigned int seed) {
  next = seed;
}

int abs(int x) {
  return (x < 0 ? -x : x);
}

int atoi(const char* nptr) {
  int x = 0;
  while (*nptr == ' ') { nptr ++; }
  while (*nptr >= '0' && *nptr <= '9') {
    x = x * 10 + *nptr - '0';
    nptr ++;
  }
  return x;
}
static uintptr_t heap_cur = 0;//uintptr_t：无符号整型，能完整存放一个指针的数值
void *malloc(size_t size) {
  // On native, malloc() will be called during initializaion of C runtime.
  // Therefore do not call panic() here, else it will yield a dead recursion:
  //   panic() -> putchar() -> (glibc) -> malloc() -> panic()
  // 初始化时设置为堆区起始地址
  if(heap_cur == 0){
    heap_cur = (uintptr_t)heap.start;
  }
  // 内存对齐到 8 字节加 7 是为了把“向下对齐”变成“向上对齐”
  size = (size + 7) & ~ 7;
  // 检查是否超过堆区
  if (heap_cur + size > (uintptr_t)heap.end) {
    return NULL;  // 堆满了，返回 NULL
  }
  // 返回当前地址，并更新分配指针
  void *ret = (void *)heap_cur;
  heap_cur += size;
  return ret;
}

void free(void *ptr) {
}

#endif
