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

default: $(BIN)

$(shell mkdir -p $(BUILD_DIR))  

# SRC_AUTO_BIND = $(abspath $(BUILD_DIR)/auto_bind.cpp) 
# $(SRC_AUTO_BIND): $(NXDC_FILES)
# 	python3 $(NVBOARD_HOME)/scripts/auto_pin_bind.py $^ $@ 
#$(YSYX_HOME)/npc/vsrc -name "*.v"
# project source
##VSRCS = $(shell find $(abspath ./vsrc) -name "*.v") \
		$(shell find $(abspath ../ysyxSoC/perip) -name "*.v") \
		$(shell find $(abspath $(YSYX_HOME)/ysyxSoC/ready-to-run/D-stage) -name "*.v")
VSRCS = $(shell find $(YSYX_HOME)/npc/vsrc -name "*.v") \
        $(shell find $(YSYX_HOME)/ysyxSoC/perip -name "*.v") \
        $(shell find $(YSYX_HOME)/ysyxSoC/ready-to-run/D-stage -name "*.v")

CSRCS = $(shell find $(abspath ./csrc) -name "*.c" -or -name "*.cc" -or -name "*.cpp") 
# 所有c++文件，其实就是sim_main.cpp和auto_bind.cpp
#CSRCS += $(SRC_AUTO_BIND)


#include $(NVBOARD_HOME)/scripts/nvboard.mk 


INCFLAGS = $(addprefix -I, $(INC_PATH))
CXXFLAGS += $(INCFLAGS) -DTOP_NAME="\"V$(TOPNAME)\""
# CXXFLAGS += $(INCFLAGS) -I $(YSYX_HOME)/nemu/tools/capstone/repo/include -DTOP_NAME="\"V$(TOPNAME)\""
LDFLAGS += -L $(YSYX_HOME)/nemu/tools/capstone/repo/ -lcapstone -lSDL2 -lSDL2_image
# LDFLAGS += -lSDL2 -lSDL2_image

#IMAGE ?= $(YSYX_HOME)/npc/new2.bin
#IMAGE ?= /home/huang/test_our/new.bin
IMAGE ?= $(YSYX_HOME)/ysyxSoC/ready-to-run/D-stage/hello-minirv-ysyxsoc.bin
# IMAGE ?= $(YSYX_HOME)/ysyxSoC/ready-to-run/D-stage/new.bin
$(BIN): $(VSRCS) $(CSRCS) #$(NVBOARD_ARCHIVE)
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

run: $(BIN)
	./$(BIN) $(IMAGE) +trace
	
clean:
	rm -rf $(BUILD_DIR) wave.vcd

.PHONY: default all clean  sim  # 添加sim到伪目标列表


