#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
//#include <nvboard.h> 
#include "VysyxSoCFull.h"  
#include "verilated.h"
 //#include "verilated_vcd_c.h" // 可选，如果要导出vcd则需要加上
static uint64_t count = 0;//计算周期数用的，可删
static FILE *itrace_fp = NULL;

#define FLASH_SIZE (16 * 1024 * 1024) // 16MB
#define FLASH_BASE 0x30000000
static uint8_t flash_mem[FLASH_SIZE] __attribute__((aligned(4096))) = {0};

// 声明外部存储器函数
extern "C" int pmem_read(int raddr);
extern "C" void init_memory(const char* path);

// define  FLASH_SIZE (16 * 1024 * 1024)
// define  FLASH_BASE 0x30000000
extern "C" void flash_init(const char *bin_path) {
    FILE *fp = fopen(bin_path, "rb");
    if (fp == NULL) {
        perror("Cannot open flash image");
        exit(1);
    }

    size_t size = fread(flash_mem, 1,FLASH_SIZE, fp);
    fclose(fp);

    printf("[flash_init] Loaded %zu bytes into flash from %s\n", size, bin_path);
    assert(size > 0);
}
extern "C" void flash_read(int32_t addr, int32_t *data) {
    // 小端方式组装 4 字节
    uint32_t val = 0;
    val |= (uint32_t)flash_mem[addr + 0];
    val |= (uint32_t)flash_mem[addr + 1] << 8;
    val |= (uint32_t)flash_mem[addr + 2] << 16;
    val |= (uint32_t)flash_mem[addr + 3] << 24;

    *data = val;
    assert(addr + 4 <= FLASH_SIZE);

    //rintf("[flash_read] addr = 0x%08x, data = 0x%08x\n", addr, *data);
}
// void disassemble(char *str,int size,uint64_t pc,uint8_t *code,int nbyte);
// void init_disasm();
// void log_mem_access(Vtop* top) {
//     if (top->do_memread) {
//         fprintf(itrace_fp, "[MEM-READ ] addr=0x%08x data=0x%08x\n",
//                 top->mem_addr, top->MemReadData);
//         fflush(itrace_fp);
//     }
//     if (top->MemWEn) {
//         fprintf(itrace_fp, "[MEM-WRITE] addr=0x%08x data=0x%08x\n",
//                 top->mem_addr, top->MemReadData);
//         fflush(itrace_fp);
//     }
// }
// 全局变量控制仿真结束
bool simulation_finished = false;
extern "C" void notify_ebreak(){
    simulation_finished = true;
}
static uint32_t *cpu_gpr = nullptr;
const char *regs[] = {
  "$0", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
  "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
  "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
  "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"
};
extern "C" void set_gpr_ptr(uint32_t *a){
    cpu_gpr = a;
}
//extern "C" void flash_read(int32_t addr, int32_t *data) { assert(0); }
void scan_registers(){
    if(!cpu_gpr) return;
    for(int i =0;i<32;i++)
    {
        printf("%s\t\t0x%08x\n",regs[i],cpu_gpr[i]);
    }
    //printf("a0\t\t0x%08x\n",top->debug_a0);
}
void single_step(VysyxSoCFull* top,VerilatedContext* contextp){//,VerilatedVcdC* tfp){
    top->clock = 0;top->eval();contextp->timeInc(1);//tfp->dump(contextp->time());
    top->clock = 1;top->eval();contextp->timeInc(1);//tfp->dump(contextp->time());
    count++;
} 

void check_trap(VysyxSoCFull* top) {
    if (simulation_finished) {
        //uint32_t a0 = top->debug_a0;
        //uint32_t pc = top->PC;
        //if (a0 == 0) {
        // 绿色 GOOD
            printf("\033[1;32mHIT GOOD TRAP\033");
        //} else {[0m at pc = 0x%08x\n", pc);
        // 红色 BAD
        //    printf("\033[1;31mHIT BAD TRAP\033[0m  at pc = 0x%08x, code = %u\n", pc, a0);
        //}
    }
}
char cmd_buf[128];
int main(int argc, char** argv, char** env) {
    const char* image_path = argv[1];
    flash_init(image_path);
    printf("argc = %d\n", argc);
    for (int i = 0; i < argc; i++) {
        printf("argv[%d] = %s\n", i, argv[i]);
    }
   // printf("MAINARGS = %s\n",MAINARGS);
    // init_disasm();
    //itrace_fp = fopen("/home/huang/ysyx-workbench/am-kernels/tests/cpu-tests/build/npc-log.txt", "w");
    //printf("itrace_fp = %p\n", (void *)itrace_fp); 
    //assert(itrace_fp);
    VerilatedContext* contextp = new VerilatedContext;
    contextp->commandArgs(argc, argv);
    VysyxSoCFull* top = new VysyxSoCFull{contextp};//创建一个Vtop实例，Vtop是你的顶层Verilog模块的C++表示。contextp是Verilator上下文对象，用于管理仿真。   
    
    // VerilatedVcdC* tfp = new VerilatedVcdC; //这是用于波形生成的对象
    // contextp->traceEverOn(true);
    // top->trace(tfp, 99); //这句代码和上面那句代码用于启用波形跟踪和连接波形对象。
    // tfp->open("wave.vcd"); //这是用于打开波形文件的代码


    // reset
    top->reset  = 1;
    top->eval(); 
    top->clock = 0;
    int cycle_count = 0;


// 给复位至少 5~10 个完整周期
    for (int i = 0; i < 10; i++) {
        top->clock = 1;
        top->eval();
        contextp->timeInc(1);

        top->clock = 0;
        top->eval();
        contextp->timeInc(1);
    }
    top->reset = 0;

    // while(!contextp->gotFinish()){ 
    //     printf("(npc) ");
    //     fflush(stdout);
    //     if(fgets(cmd_buf,sizeof(cmd_buf),stdin) == NULL){
    //         break;
    //     }
    //     cmd_buf[strcspn(cmd_buf, "\n")] = '\0';
        
    //     if (simulation_finished &&
    //         (strncmp(cmd_buf, "si", 2) == 0 || strcmp(cmd_buf, "c") == 0)) {
    //         printf("Program execution has ended. To restart the program, exit NPC and run again.\n");
    //         continue;          // 忽略 si / c
    //     }

    //     if(strcmp(cmd_buf,"q") == 0){
    //         break;
    //     }else if(strncmp(cmd_buf,"si",2) == 0){
    //         int n = 1;
    //         sscanf(cmd_buf,"si %d",&n);
    //         for(int i =0;i<n && !simulation_finished;i++)
    //         {

                
    //             // char disasm_output [128];
    //             // uint32_t inst = top->inst;
    //             // uint8_t code[4];
    //             // code[0] = inst & 0xff;
    //             // code[1] = (inst >> 8) & 0xff;
    //             // code[2] = (inst >> 16) & 0xff;
    //             // code[3] = (inst >> 24) & 0xff;
    //             // disassemble(disasm_output,sizeof(disasm_output),top->PC,code,4);
    //             // printf("0x%08x: 0x%08x %s\n",top->PC,top->inst,disasm_output);
    //             // fprintf(itrace_fp, "0x%08x: 0x%08x %s\n", top->PC, top->inst, disasm_output);
    //             // fflush(itrace_fp);   /* 强制把缓冲区刷到磁盘，先调试用 */
    //             //printf("[itrace] write pc=0x%08x\n", top->PC);  /* 终端能看到就说明确实执行了 */
    //             //log_mem_access(top);
    //             single_step(top,contextp);//,tfp);
    //         }
    //         // printf("PC   = 0x%08x\n",top->PC);
    //         // printf("inst = 0x%08x\n",top->inst);
    //         check_trap(top);
    //     }else if(strcmp(cmd_buf,"c") == 0)
    //     {
    //         while(!simulation_finished){

    //             // char disasm_output [128];
    //             // uint32_t inst = top->inst;
    //             // uint8_t code[4];
    //             // code[0] = inst & 0xff;
    //             // code[1] = (inst >> 8) & 0xff;
    //             // code[2] = (inst >> 16) & 0xff;
    //             // code[3] = (inst >> 24) & 0xff;
    //             // disassemble(disasm_output,sizeof(disasm_output),top->PC,code,4);
    //             // printf("0x%08x: 0x%08x %s\n",top->PC,top->inst,disasm_output);
    //             // fprintf(itrace_fp, "0x%08x: 0x%08x %s\n", top->PC, top->inst, disasm_output);
    //             // fflush(itrace_fp);
    //             //log_mem_access(top);
    //             single_step(top,contextp);//,tfp);
    //         }
    //         check_trap(top);
    //     }else if(strcmp(cmd_buf,"info r") == 0) {
    //         //printf("PC = 0x%08x\n", top->PC);
    //         scan_registers();            
    //     }else if (strncmp(cmd_buf,"x",1)==0)
    //     {
    //         int N =0;
    //         uint32_t addr =0;
    //         if(sscanf(cmd_buf,"x %d %x",&N,&addr) == 2) {
    //             for(int i =0;i<N;i++){
    //                 uint32_t cur_addr = addr + i*4;
    //                 uint32_t data = pmem_read(cur_addr);
    //                 printf("0x%08x: 0x%08x\n", cur_addr, data);
    //             }
    //         }else {
    //             printf("Usage: x N ADDR\n");
    //         }
    //     } else {
    //         printf("Unknown command: %s\n", cmd_buf);
    //     }
    // }
            while(!simulation_finished){

                // char disasm_output [128];
                // uint32_t inst = top->inst;
                // uint8_t code[4];
                // code[0] = inst & 0xff;
                // code[1] = (inst >> 8) & 0xff;
                // code[2] = (inst >> 16) & 0xff;
                // code[3] = (inst >> 24) & 0xff;
                // disassemble(disasm_output,sizeof(disasm_output),top->PC,code,4);
                // printf("0x%08x: 0x%08x %s\n",top->PC,top->inst,disasm_output);
                // fprintf(itrace_fp, "0x%08x: 0x%08x %s\n", top->PC, top->inst, disasm_output);
                // fflush(itrace_fp);
                //log_mem_access(top);
                single_step(top,contextp);//,tfp);
            }
            check_trap(top);

    //fclose(itrace_fp);
    printf("Total cycles: %llu\n", (unsigned long long)count);//计算周期数用的，可删
    delete top;
    //tfp->close();//这是用于关闭波形文件的代码
    delete contextp;
    return 0;
}