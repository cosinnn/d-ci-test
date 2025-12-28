// #include <capstone/capstone.h>
// #include <assert.h>
// #include <stdio.h>
// #include <stdint.h>
// #include <dlfcn.h>

// static csh handle;
// static void *capstone_lib = NULL;

// static cs_err (*cs_open_dl)(cs_arch, cs_mode, csh *) = NULL;
// static size_t (*cs_disasm_dl)(csh handle,const uint8_t *code,
//     size_t code_size,uint64_t address,size_t count,cs_insn **insn);
// static void(*cs_free_dl)(cs_insn *insn,size_t count);

// void init_disasm(){

//     capstone_lib = dlopen("/home/huang/ysyx-workbench/nemu/tools/capstone/repo/libcapstone.so.5", RTLD_LAZY);
//     assert(capstone_lib);

//     cs_open_dl   = (cs_err (*)(cs_arch, cs_mode, csh *))
//                     dlsym(capstone_lib, "cs_open");
//     cs_disasm_dl = (size_t (*)(csh, const uint8_t *, size_t,
//                            uint64_t, size_t, cs_insn **))
//                     dlsym(capstone_lib, "cs_disasm");
//     cs_free_dl   = (void (*)(cs_insn *, size_t))
//                     dlsym(capstone_lib, "cs_free");

// assert(cs_open_dl);
// assert(cs_disasm_dl);
// assert(cs_free_dl);

//     cs_arch arch = CS_ARCH_RISCV;
//     cs_mode mode = CS_MODE_RISCV32;
// #ifdef CONFIG_ISA64
//     mode = CS_MODE_RISCV64;
// #endif
//     int ret = cs_open_dl(arch,mode,&handle);
//     assert(ret == CS_ERR_OK);
// }

// void disassemble(char *str,int size,uint64_t pc,uint8_t *code,int nbyte){
//     cs_insn *insn;
//     size_t count = cs_disasm_dl(handle,code,nbyte,pc,0,&insn);
//     if (count != 1) {
//         printf("Error: Failed to disassemble instruction at PC = 0x%08lx, count = %zu\n", pc, count);
//         for (int j = 0; j < 4; j++) {
//             printf("0x%02x ", code[j]);
//         }
//         printf("\n");
//         return;
//     }

//     assert(count == 1);
//     int ret = snprintf(str,size,"%s",insn->mnemonic);
//     if(insn->op_str[0] != '\0'){
//         snprintf(str+ret,size-ret,"\t%s",insn->op_str);
//     }
//     cs_free_dl(insn,count);
// }
