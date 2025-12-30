TOPNAME = ysyxSoCFull
# NXDC_FILES = constr/top.nxdc
INC_PATH ?=

VERILATOR = verilator
VERILATOR_CFLAGS += -MMD --build --cc --trace  \
				-O3 --x-assign fast --x-initial fast --noassert \
				--timescale "1ns/1ns" --no-timing
## 新增：把两个 rtl 目录加入 Verilog 的 `include` 搜索路径	
VERILATOR_CFLAGS += -I$(abspath ../ysyxSoC/perip/uart16550/rtl) -I$(abspath ../ysyxSoC/perip/spi/rtl)
BUILD_DIR = ./build
OBJ_DIR = $(BUILD_DIR)/obj_dir
BIN = $(BUILD_DIR)/$(TOPNAME)   

# 定义测试程序路径
HELLO_IMAGE = $(YSYX_HOME)/ysyxSoC/ready-to-run/D-stage/hello-minirv-ysyxsoc.bin
NEW_IMAGE = $(YSYX_HOME)/ysyxSoC/ready-to-run/D-stage/new.bin
# 默认使用 hello 程序
IMAGE ?= $(HELLO_IMAGE)

default: $(BIN)

$(shell mkdir -p $(BUILD_DIR))  

# SRC_AUTO_BIND = $(abspath $(BUILD_DIR)/auto_bind.cpp) 
# $(SRC_AUTO_BIND): $(NXDC_FILES)
# 	python3 $(NVBOARD_HOME)/scripts/auto_pin_bind.py $^ $@ 
#$(YSYX_HOME)/npc/vsrc -name "*.v"
# project source
VSRCS = $(shell find $(YSYX_HOME)/npc/vsrc -name "*.v") \
        $(shell find $(YSYX_HOME)/ysyxSoC/perip -name "*.v") \
        $(shell find $(YSYX_HOME)/ysyxSoC/ready-to-run/D-stage -name "*.v")

CSRCS = $(shell find $(abspath ./csrc) -name "*.c" -or -name "*.cc" -or -name "*.cpp") 

INCFLAGS = $(addprefix -I, $(INC_PATH))
CXXFLAGS += $(INCFLAGS) -DTOP_NAME="\"V$(TOPNAME)\""
LDFLAGS += -lSDL2 -lSDL2_image

# # 构建测试程序的规则
# build-test:
# ifdef ALL
# ifeq ($(ALL),hello)
# 	@echo "Using pre-built hello program"
# else
# 	@echo "Building test program: $(ALL)"
# 	cd $(YSYX_HOME)/am-kernels/tests/cpu-tests && \
# 	make ARCH=minirv-npc ALL=$(ALL)
# 	cd $(YSYX_HOME)/ysyxSoC/ready-to-run/D-stage && \
# 	bash gen.sh $(YSYX_HOME)/am-kernels/tests/cpu-tests/build/$(ALL)-minirv-npc.elf
# endif
# endif

$(BIN): $(VSRCS) $(CSRCS)
	@rm -rf $(OBJ_DIR)
	$(VERILATOR) $(VERILATOR_CFLAGS) \
		--top-module $(TOPNAME) $^ \
		$(addprefix -CFLAGS , $(CXXFLAGS)) $(addprefix -LDFLAGS , $(LDFLAGS)) \
		--Mdir $(OBJ_DIR) --exe -o $(abspath $(BIN))

all: default

sim: $(BIN)
	@$(call git_commit, "sim RTL") 
	@echo "Running simulation with git commit..."
	./build/top +trace

# 修改 run 规则
run: $(BIN) build-test
	@echo "Running simulation..."
	./$(BIN) $(IMG) +trace
	
clean:
	rm -rf $(YSYX_HOME)/npc/build

clean-trace:

clean-test:
	cd $(YSYX_HOME)/am-kernels/tests/cpu-tests && make clean
	rm -f $(NEW_IMAGE)

.PHONY: default all clean sim run build-test clean-test