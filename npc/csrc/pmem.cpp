#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <cstring>
#include <elf.h>
#define MEM_SIZE (32 * 1024 * 1024) // 1MB存储器
static uint32_t memory[MEM_SIZE / 4]; // 32位存储器
#define MEM_BASE 0x80000000
#define RTC_ADDR 0xa0000048
#define SERIAL_PORT 0xa0000038
static uint64_t nowtime = 0;
uint64_t get_system_time_us() {
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) {
        perror("gettimeofday failed");
        return 0;  
    }
    return (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;
}

extern "C" void init_memory(const char* path) {
    for (int i = 0; i < MEM_SIZE / 4; i++) {
        memory[i] = 0;
    } 
    FILE* fp = fopen(path, "rb");  // 二进制模式
    if (!fp) {
        perror("Failed to open file");
        exit(1);
    }
    size_t bytes_read = fread(memory, 1, MEM_SIZE, fp);  // 读取字节流
    fclose(fp);
        uint32_t first_inst = memory[0];
    printf("First instruction (word) = 0x%08x\n", first_inst);
}
// 存储器读取函数
extern "C" int pmem_read(int raddr) {
    // ----------MMIO-----------
    if(raddr == RTC_ADDR){
        nowtime = get_system_time_us();
        return (uint32_t) nowtime;
    }else if(raddr == RTC_ADDR + 4){
        return (uint32_t) (nowtime >> 32);
    }
    raddr = raddr - MEM_BASE;
    // 添加边界检查
    if (raddr < 0 || raddr >= MEM_SIZE) {
        return 0;
    }
    
    // 对齐地址到字边界
    uint32_t aligned_addr = raddr & ~0x3; 
    return memory[aligned_addr >> 2];
}

// 增强的存储器写入函数
extern "C" void pmem_write(int waddr, int wdata, int wmask_int) {
        // ---------- MMIO 处理 ----------
    if (waddr == SERIAL_PORT) {
        char ch = (char)(wdata & 0xFF);
        putchar(ch);   
        fflush(stdout);
        return;
    }
    uint8_t wmask = wmask_int & 0xF;
    waddr = waddr - MEM_BASE;
    uint32_t aligned_addr = waddr & ~0x3;//将地址的最低2位强制设为0，实现向下取整到最近的4字节边界。
    uint32_t index = aligned_addr >> 2;

    // 检查地址是否在有效范围内
    if (index >= MEM_SIZE / 4) {
        printf("Error: Memory write out of bounds at address 0x%08x\n", waddr);
        return;
    }

    uint8_t* mem_byte = (uint8_t*)&memory[index];


    // 应用字节掩码
    if (wmask & 0x1) mem_byte[0] = wdata & 0xFF;
    if (wmask & 0x2) mem_byte[1] = (wdata >> 8) & 0xFF;
    if (wmask & 0x4) mem_byte[2] = (wdata >> 16) & 0xFF;
    if (wmask & 0x8) mem_byte[3] = (wdata >> 24) & 0xFF;
    //     uint32_t target_addr = 0x80008FFC;
    // if ((waddr + MEM_BASE) == target_addr) {
    //     printf("[DEBUG] Write mem[0x%08x] = 0x%08x (wmask=0x%x)\n",
    //            waddr + MEM_BASE, wdata, wmask);
    //     // 再读回验证是否写入成功：
    //     uint32_t readback = memory[index];
    //     printf("[DEBUG] After write, mem[0x%08x] now = 0x%08x\n", target_addr, readback);
    //     }

}