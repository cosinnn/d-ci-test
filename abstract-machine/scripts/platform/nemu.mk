AM_SRCS := platform/nemu/trm.c \
           platform/nemu/ioe/ioe.c \
           platform/nemu/ioe/timer.c \
           platform/nemu/ioe/input.c \
           platform/nemu/ioe/gpu.c \
           platform/nemu/ioe/audio.c \
           platform/nemu/ioe/disk.c \
           platform/nemu/mpe.c
# AM_SRCS 用于存储所有需要编译的源文件路径,这些源文件位于platform/nemu和其子目录中将被编译成目标文件(.o文件），最终链接成一个可执行文件

CFLAGS    += -fdata-sections -ffunction-sections
CFLAGS    += -I$(AM_HOME)/am/src/platform/nemu/include
LDSCRIPTS += $(AM_HOME)/scripts/linker.ld
LDFLAGS   += --defsym=_pmem_start=0x80000000 --defsym=_entry_offset=0x0
LDFLAGS   += --gc-sections -e _start
NEMUFLAGS += -l $(shell dirname $(IMAGE).elf)/nemu-log.txt -b
#CFLAGS: 这是一个变量，用于存储编译器的标志。
#LDSCRIPTS：这是一个变量，用于存储链接脚本文件路径。
#LDFLAGS: 这是一个变量，用于存储链接器的标志。--defsym：这是一个链接器选项，用于定义一个符号及其值.
#_pmem_start：这是定义的符号名称，通常用于表示某个内存区域的起始地址。_entry_offset：这是定义的符号名称，通常用于表示程序入口点的偏移量
#--gc-sections:链接器的一个选项.作用是:去除未使用的代码段和数据段,优化生成的可执行文件大小,-e:用于指定程序的入口点。
#NEMUFLAGS: 用于存储运行 NEMU 时的参数.dirname 是一个命令行工具，用于提取文件路径中的目录部分

MAINARGS_MAX_LEN = 64
MAINARGS_PLACEHOLDER = the_insert-arg_rule_in_Makefile_will_insert_mainargs_here
CFLAGS += -DMAINARGS_MAX_LEN=$(MAINARGS_MAX_LEN) -DMAINARGS_PLACEHOLDER=$(MAINARGS_PLACEHOLDER)

insert-arg: image
	@python3 $(AM_HOME)/tools/insert-arg.py $(IMAGE).bin $(MAINARGS_MAX_LEN) $(MAINARGS_PLACEHOLDER) "$(mainargs)"

image: image-dep
	@$(OBJDUMP) -d $(IMAGE).elf > $(IMAGE).txt
	@echo + OBJCOPY "->" $(IMAGE_REL).bin
	@$(OBJCOPY) -S --set-section-flags .bss=alloc,contents -O binary $(IMAGE).elf $(IMAGE).bin
# objdump 是一个用于显示可执行文件内容的工具。-d：这是 objdump 的一个选项，表示反汇编代码段.>：这是一个重定向操作符，将命令的输出重定向到文件中,而不是打印到终端屏幕上。
# -S：这是 objcopy 的一个选项，表示去除符号表。alloc：表示该段在运行时需要分配内存。contents：表示该段的内容需要被加载到内存中-O binary：将输出文件转换为二进制格式
run: insert-arg
	$(MAKE) -C $(NEMU_HOME) ISA=$(ISA) run ARGS="$(NEMUFLAGS)" IMG=$(IMAGE).bin
# -C：make的一个选项，表示切换到指定的目录后再执行后续的命令。
gdb: insert-arg
	$(MAKE) -C $(NEMU_HOME) ISA=$(ISA) gdb ARGS="$(NEMUFLAGS)" IMG=$(IMAGE).bin

.PHONY: insert-arg
