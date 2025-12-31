`define ALU_ADD 4'b0000  // 加法
`define ALU_SUB 4'b0001  // 减法
`define ALU_AND 4'b0010  // 按位与
`define ALU_OR 4'b0011  // 按位或
`define ALU_XOR 4'b0100  // 按位异或
`define ALU_SLL 4'b0101  // 逻辑左移
`define ALU_SRL 4'b0110  // 逻辑右移
`define ALU_SRA 4'b0111  // 算术右移
`define ALU_SLT 4'b1000  // 有符号小于比较
`define ALU_SLTU 4'b1001  // 无符号小于比较
`define ALU_LUI 4'b1010  // 仅输出B (LUI指令使用)

// ALU操作数选择信号定义
`define SRC_REG 2'b00    // 使用寄存器值
`define SRC_IMM 2'b01    // 使用立即数
`define SRC_PC 2'b01    // 使用PC值
`define SRC_ZERO 2'b10    // 使用常数0
`define SRC_FOUR 2'b10    // 使用常数4

// 分支类型定义
`define BRANCH_BEQ 3'b000  // 相等分支
`define BRANCH_BNE 3'b001  // 不相等分支
`define BRANCH_BLT 3'b100  // 小于分支
`define BRANCH_BGE 3'b101  // 大于等于分支
`define BRANCH_BLTU 3'b110  // 无符号小于分支
`define BRANCH_BGEU 3'b111  // 无符号大于等于分支

// 常量定义
`define CONSTANT_FOUR 32'h4         // 常数4
`define JALR_MASK 32'hFFFFFFFE  // JALR指令清零最低位的掩码

// funct7常量定义
`define FUNCT7_STD 7'b0000000       // 标准操作
`define FUNCT7_ALT 7'b0100000       // 替代操作(SUB/SRA等)

// CSR寄存器地址定义
`define CSR_MVENDORID 12'hF11        // 厂商ID寄存器
`define CSR_MARCHID 12'hF12          // 架构ID寄存器
`define CSR_MSTATUS 12'h300          // 机器状态寄存器
`define CSR_MTVEC 12'h305          // 机器陷阱向量寄存器
`define CSR_MEPC 12'h341          // 机器异常程序计数器
`define CSR_MCAUSE 12'h342          // 机器异常原因寄存器
`define CSR_MCYCLE 12'hB00          // 机器周期计数器低32位
`define CSR_MCYCLEH 12'hB80          // 机器周期计数器高32位

// 异常原因编码
`define ECALL_M_MODE 32'd11           // 机器模式下的环境调用

// 访存掩码和数据对齐定义说明
// 读掩码：直接使用，无需偏移，但读取的数据需要根据地址提取正确的字节/半字
// 写掩码：需要根据地址进行偏移计算（使用case语句实现，避免移位运算符）
//   SB(字节): addr[1:0]=00→4'b0001, 01→4'b0010, 10→4'b0100, 11→4'b1000
//   SH(半字): addr[1]=0→4'b0011, addr[1]=1→4'b1100
//   SW(全字): 掩码4'b1111，无需偏移
// 写数据：需要根据地址进行对齐处理（使用明确的位拼接，避免移位运算符）
//   SB(字节): addr[1:0]=00→{24'b0,data[7:0]}, 01→{16'b0,data[7:0],8'b0}等
//   SH(半字): addr[1]=0→{16'b0,data[15:0]}, addr[1]=1→{data[15:0],16'b0}
//   SW(全字): 数据不需要偏移
// 读数据：需要根据地址从32位数据中提取正确的字节/半字
//   LB/LBU: 根据addr[1:0]选择字节位置 (7:0, 15:8, 23:16, 31:24)
//   LH/LHU: 根据addr[1]选择半字位置 (15:0, 31:16)
//   LW: 直接使用32位数据

// 学号相关常量定义
`define MVENDORID_VALUE 32'h79737978  // "ysyx"的ASCII码
`define MARCHID_VALUE 32'd24090003

// 系统指令编码
`define MRET_INST 12'h302          // MRET指令编码 (0011000 00010)


module ysyx_25080202 (
    input clock,
    input reset,

    // IFU memory interface
    output        io_ifu_reqValid,
    output [31:0] io_ifu_addr,
    input         io_ifu_respValid,
    input  [31:0] io_ifu_rdata,

    // LSU memory interface
    output        io_lsu_reqValid,
    output [31:0] io_lsu_addr,
    output        io_lsu_wen,
    output [31:0] io_lsu_wdata,
    output [ 3:0] io_lsu_wmask,
    output [ 1:0] io_lsu_size,
    input         io_lsu_respValid,
    input  [31:0] io_lsu_rdata
);
  // Clock and reset internal signals
  wire i_clk;
  wire i_rst;

  // Assign top-level ports to internal signals
  assign i_clk = clock;
  assign i_rst = reset;

  // Internal interface signals
  wire [31:0] o_lsu_addr;
  wire [31:0] o_lsu_wdata;
  wire o_lsu_wen;
  wire o_lsu_en;
  wire [3:0] o_lsu_wmask;
  wire [1:0] o_lsu_size;
  wire [31:0] i_lsu_rdata;
  wire i_lsu_respValid;
  wire [31:0] o_ifu_raddr;
  wire o_inst_valid;
  wire o_ifu_reqValid;
  wire [31:0] i_ifu_rdata;
  wire i_ifu_respValid;

  // Assign internal signals to top-level io ports
  assign io_ifu_reqValid = o_ifu_reqValid;
  assign io_ifu_addr = o_ifu_raddr;
  assign i_ifu_respValid = io_ifu_respValid;
  assign i_ifu_rdata = io_ifu_rdata;
  assign io_lsu_reqValid = o_lsu_en;
  assign io_lsu_addr = o_lsu_addr;
  assign io_lsu_wen = o_lsu_wen;
  assign io_lsu_wdata = o_lsu_wdata;
  assign io_lsu_wmask = o_lsu_wmask;
  assign io_lsu_size = o_lsu_size;
  assign i_lsu_respValid = io_lsu_respValid;
  assign i_lsu_rdata = io_lsu_rdata;

  // IFU related signals
  wire [31:0] w_pc;
  wire [31:0] w_inst;
  wire        w_inst_valid;  // Instruction valid signal
  wire        w_ifu_reqValid;  // IFU request valid signal
  wire [31:0] w_next_pc;
  wire        w_pc_update_en;
  wire [ 3:0] w_ifu_state;  // IFU状态信号

  // IDU related signals
  wire [ 4:0] w_rs1_addr;
  wire [ 4:0] w_rs2_addr;
  wire [ 4:0] w_rd_addr;
  wire [ 6:0] w_opcode;
  wire [ 2:0] w_funct3;
  wire [ 6:0] w_funct7;
  wire [31:0] w_imm;

  wire        w_lsu_wen;
  wire [ 3:0] w_lsu_wmask;
  wire [ 3:0] w_lsu_rmask;

  // Other control signals
  wire        w_reg_wen;
  wire        w_jump;

  // Control signals from IDU to EXU
  wire [ 3:0] w_alu_op;
  wire [ 1:0] w_alu_src1_sel;
  wire [ 1:0] w_alu_src2_sel;
  wire        w_branch;
  wire        w_jalr;

  // CSR related signals
  wire        w_ecall;
  wire        w_csr_we;
  wire [11:0] w_csr_addr;
  wire [ 1:0] w_csr_op;
  wire [31:0] w_csr_rdata;
  wire [31:0] w_csr_wdata;
  wire        w_mret;

  // EXU related
  wire [31:0] w_alu_result;
  wire        w_exu_pc_update;

  // LSU related
  wire [31:0] w_lsu_rdata;
  wire        w_load_inst;  // Load指令信号
  wire        w_store_inst;  // Store指令信号
  wire        w_load_reg_wen;  // Load写回寄存器使能信号
  wire        w_load_done;  // Load操作完成信号
  wire        w_store_done;  // Store操作完成信号
  wire        w_lsu_reqValid;  // LSU请求有效信号

  // WBU related
  wire [31:0] w_rs1_data;
  wire [31:0] w_rs2_data;
  wire [31:0] w_rd_wdata;
  wire [31:0] w_mtvec;
  wire [31:0] w_mepc;

  // PC update signal comes directly from EXU
  assign w_pc_update_en = w_exu_pc_update;
  assign o_inst_valid = w_inst_valid;
  assign o_ifu_reqValid = w_ifu_reqValid;
  // LSU request valid signal to computer
  assign o_lsu_en = w_lsu_reqValid;
  // Instantiate IFU with SimpleBus interface
  ysyx_24090003_IFU ifu (
      .i_clk(i_clk),
      .i_rst(i_rst),
      .i_next_pc(w_next_pc),
      .i_pc_update_en(w_pc_update_en),
      .i_load_inst(w_load_inst),
      .i_store_inst(w_store_inst),
      .i_load_done(w_load_done),
      .i_store_done(w_store_done),
      .o_inst(w_inst),
      .o_pc(w_pc),
      .o_inst_valid(w_inst_valid),
      .o_ifu_reqValid(w_ifu_reqValid),
      .o_ifu_raddr(o_ifu_raddr),
      .i_ifu_rdata(i_ifu_rdata),
      .i_ifu_respValid(i_ifu_respValid),
      .o_ifu_state(w_ifu_state)
  );

  // Instantiate IDU
  ysyx_24090003_IDU idu (
      .i_inst(w_inst),
      .i_inst_valid(w_inst_valid),
      .i_rst(i_rst),
      .o_rs1_addr(w_rs1_addr),
      .o_rs2_addr(w_rs2_addr),
      .o_rd_addr(w_rd_addr),
      .o_opcode(w_opcode),
      .o_funct3(w_funct3),
      .o_funct7(w_funct7),
      .o_imm(w_imm),
      .o_lsu_wen(w_lsu_wen),
      .o_lsu_wmask(w_lsu_wmask),
      .o_lsu_rmask(w_lsu_rmask),
      .o_reg_wen(w_reg_wen),
      .o_jump(w_jump),
      .o_alu_op(w_alu_op),
      .o_alu_src1_sel(w_alu_src1_sel),
      .o_alu_src2_sel(w_alu_src2_sel),
      .o_branch(w_branch),
      .o_jalr(w_jalr),
      .o_ecall(w_ecall),
      .o_csr_we(w_csr_we),
      .o_csr_addr(w_csr_addr),
      .o_csr_op(w_csr_op),
      .o_mret(w_mret)
  );

  // Instantiate EXU
  ysyx_24090003_EXU exu (
      .i_pc(w_pc),
      .i_imm(w_imm),
      .i_rs1_data(w_rs1_data),
      .i_rs2_data(w_rs2_data),
      .i_alu_op(w_alu_op),
      .i_alu_src1_sel(w_alu_src1_sel),
      .i_alu_src2_sel(w_alu_src2_sel),
      .i_jump(w_jump),
      .i_jalr(w_jalr),
      .i_branch(w_branch),
      .i_funct3(w_funct3),
      .i_funct7(w_funct7),
      .i_ecall(w_ecall),
      .i_csr_rdata(w_csr_rdata),
      .i_mtvec(w_mtvec),
      .i_mret(w_mret),
      .i_mepc(w_mepc),
      .o_alu_result(w_alu_result),
      .o_next_pc(w_next_pc),
      .o_pc_update(w_exu_pc_update),
      .o_csr_wdata(w_csr_wdata)
  );

  // Instantiate LSU
  ysyx_24090003_LSU lsu (
      .i_clk(i_clk),
      .i_rst(i_rst),
      .i_lsu_addr(w_alu_result),
      .i_lsu_wdata(w_rs2_data),
      .i_lsu_wen(w_lsu_wen),
      .i_lsu_wmask(w_lsu_wmask),
      .i_lsu_rmask(w_lsu_rmask),
      .i_lsu_rdata(i_lsu_rdata),
      .i_lsu_respValid(i_lsu_respValid),  // 添加respValid信号

      // 传递IFU状态信号给LSU
      .i_ifu_inst_valid(w_inst_valid),
      .i_ifu_state(w_ifu_state),

      .o_lsu_addr(o_lsu_addr),
      .o_lsu_wdata(o_lsu_wdata),
      .o_lsu_wen(o_lsu_wen),
      .o_lsu_reqValid(w_lsu_reqValid),
      .o_lsu_wmask(o_lsu_wmask),
      .o_lsu_size(o_lsu_size),
      .o_lsu_rdata(w_lsu_rdata),
      .o_load_inst(w_load_inst),
      .o_store_inst(w_store_inst),
      .o_load_reg_wen(w_load_reg_wen),
      .o_load_done(w_load_done),
      .o_store_done(w_store_done)
  );

  // Instantiate WBU
  ysyx_24090003_WBU wbu (
      .i_clk(i_clk),
      .i_rst(i_rst),
      .i_rd_addr(w_rd_addr),
      .i_rs1_addr(w_rs1_addr),
      .i_rs2_addr(w_rs2_addr),
      .i_alu_result(w_alu_result),
      .i_mem_rdata(w_lsu_rdata),
      .i_pc(w_pc),
      .i_reg_wen(w_reg_wen),
      .i_jump(w_jump),
      .i_load_reg_wen(w_load_reg_wen),
      .i_csr_we(w_csr_we),
      .i_csr_addr(w_csr_addr),
      .i_csr_op(w_csr_op),
      .i_csr_wdata(w_csr_wdata),
      .i_ecall(w_ecall),
      .i_mret(w_mret),
      .o_rs1_data(w_rs1_data),
      .o_rs2_data(w_rs2_data),
      .o_rd_wdata(w_rd_wdata),
      .o_csr_rdata(w_csr_rdata),
      .o_mtvec(w_mtvec),
      .o_mepc(w_mepc)
  );

endmodule
module ysyx_24090003_IFU (
    input         i_clk,
    input         i_rst,
    input  [31:0] i_next_pc,
    input         i_pc_update_en,
    input         i_load_inst,      // Load指令信号（用于状态转移判断）
    input         i_store_inst,     // Store指令信号（用于状态转移判断）
    input         i_load_done,      // Load操作完成信号（从LSU传入）
    input         i_store_done,     // Store操作完成信号（从LSU传入）
    output [31:0] o_inst,
    output [31:0] o_pc,
    output        o_inst_valid,     // Indicates when instruction is valid
    output        o_ifu_reqValid,   // IFU request valid signal
    output [31:0] o_ifu_raddr,      // Address to memory
    input  [31:0] i_ifu_rdata,      // Data from memory
    input         i_ifu_respValid,  // IFU response valid signal
    output [ 3:0] o_ifu_state       // IFU状态输出
);

  localparam STATE_ADDR = 4'b0000;  // Address phase - send address request
  localparam STATE_WAIT = 4'b0001;  // Wait for memory response and receive data when respValid
  localparam STATE_UPDATE = 4'b0010;  // PC update state - update PC after decode/execute
  localparam STATE_LOAD_WAIT = 4'b0011;  // Wait for load instruction to complete
  localparam STATE_STORE_WAIT = 4'b0100;  // Wait for store instruction to complete

  // Registers
  reg  [31:0] r_pc;  // Current PC
  reg  [31:0] r_pc_inst;  // PC corresponding to current instruction
  reg  [31:0] r_inst;  // Instruction register
  reg  [ 3:0] r_state;  // Current state register (now 4 bits)

  // Next state logic (combinational)
  reg  [ 3:0] next_state;  // Next state (now 4 bits)

  // Output registers (sequential)
  reg         r_inst_valid;  // Output register: instruction valid
  reg         r_ifu_reqValid;  // Output register: IFU request valid

  // PC+4 adder
  wire [31:0] w_pc_plus_4;
  wire        w_cout;

  adder_32bit pc_plus4_adder (
      .a(r_pc),
      .b(32'h4),
      .cin(1'b0),
      .sum(w_pc_plus_4),
      .cout(w_cout)
  );
  // 第一段：组合逻辑 - 状态转移逻辑
  always @(*) begin
    case (r_state)
      STATE_ADDR: begin
        next_state = STATE_WAIT;  // 地址阶段后进入等待响应状态
      end
      STATE_WAIT: begin
        if (i_ifu_respValid) begin
          next_state = STATE_UPDATE;  // 收到响应后直接进入更新状态
        end else begin
          next_state = STATE_WAIT;  // 继续等待响应
        end
      end

      STATE_UPDATE: begin
        if (i_load_inst) begin
          next_state = STATE_LOAD_WAIT;
        end else if (i_store_inst) begin
          next_state = STATE_STORE_WAIT;
        end else begin
          next_state = STATE_ADDR;
        end
      end
      STATE_LOAD_WAIT: begin
        if (i_load_done) begin
          next_state = STATE_ADDR;
        end else begin
          next_state = STATE_LOAD_WAIT;
        end
      end
      STATE_STORE_WAIT: begin
        if (i_store_done) begin
          next_state = STATE_ADDR;
        end else begin
          next_state = STATE_STORE_WAIT;
        end
      end

      default: begin
        next_state = STATE_ADDR;
      end
    endcase
  end
  // 第二段：时序逻辑 - 状态更新、寄存器更新和输出逻辑
  always @(posedge i_clk) begin
    if (i_rst) begin
      r_pc           <= 32'h30000000;
      r_pc_inst      <= 32'h30000000;
      r_inst         <= 32'b0;
      r_state        <= STATE_ADDR;  // 复位后直接进入地址状态
      r_inst_valid   <= 1'b0;
      r_ifu_reqValid <= 1'b0;
    end else begin
      // 状态更新
      r_state <= next_state;
      case (r_state)
        STATE_ADDR: begin
          r_pc_inst <= r_pc;  // 记录当前PC对应的指令
          r_inst_valid <= 1'b0;  // 指令还未有效
          r_ifu_reqValid <= 1'b1;  // 发出请求
        end

        STATE_WAIT: begin
          // 等待响应期间保持请求和地址稳定
          if (i_ifu_respValid) begin
            r_inst <= i_ifu_rdata;
          end else begin
            r_inst <= 32'b0;
          end
          r_inst_valid   <= i_ifu_respValid;
          r_ifu_reqValid <= 1'b0;  // 清除请求信号
        end
        STATE_UPDATE: begin
          // 只有在非load/store指令时才更新PC
          if (~i_load_inst && ~i_store_inst) begin
            if (i_pc_update_en) begin
              r_pc <= i_next_pc;
            end else begin
              r_pc <= w_pc_plus_4;
            end
            // 非load/store指令：指令处理完成，准备下一条指令
            r_inst_valid   <= 1'b0;  // 指令处理完成
            r_ifu_reqValid <= 1'b0;
          end else begin
            // load/store指令：在进入WAIT前更新PC
            r_pc <= w_pc_plus_4;
            r_inst_valid <= 1'b1;  // 保持指令有效，让LSU能处理
            r_ifu_reqValid <= 1'b0;
          end
        end
        STATE_LOAD_WAIT: begin
          r_inst_valid   <= 1'b0;
          r_ifu_reqValid <= 1'b0;
          // PC已经在UPDATE状态更新过了，这里不再更新
        end
        STATE_STORE_WAIT: begin
          r_inst_valid   <= 1'b0;
          r_ifu_reqValid <= 1'b0;
          // PC已经在UPDATE状态更新过了，这里不再更新
        end

        default: begin
          r_inst_valid   <= 1'b0;
          r_ifu_reqValid <= 1'b0;
        end
      endcase
    end
  end

  // 输出赋值
  assign o_inst_valid = r_inst_valid;
  assign o_ifu_raddr = r_pc_inst;
  assign o_ifu_state = next_state;  // 输出下一状态，使LSU能在当前周期看到即将进入的状态
  assign o_ifu_reqValid = r_ifu_reqValid;

  // Outputs
  assign o_inst = r_inst;
  assign o_pc = r_pc;

  // DPI calls for debugging
endmodule

module ysyx_24090003_IDU (
    input [31:0] i_inst,
    input i_inst_valid,  // 指令有效信号
    input i_rst,
    output [4:0] o_rs1_addr,
    output [4:0] o_rs2_addr,
    output [4:0] o_rd_addr,
    output [6:0] o_opcode,
    output [2:0] o_funct3,
    output [6:0] o_funct7,
    output [31:0] o_imm,
    output reg o_lsu_wen,
    output reg [3:0] o_lsu_wmask,  // 4位写掩码: 全字0100，半字0010，字节0001
    output reg [3:0] o_lsu_rmask,  // 4位读掩码: 全字0100，半字0010，字节0001，有符号扩展最高位为1
    output reg o_reg_wen,
    output reg o_jump,  // 跳转信号
    // 新增EXU控制信号
    output reg [3:0] o_alu_op,  // ALU操作码（4位）
    output reg [1:0] o_alu_src1_sel,  // ALU第一操作数选择（00:reg_data1, 01:pc, 10)
    output reg [1:0] o_alu_src2_sel,  // ALU第二操作数选择（00:reg_data2, 01:imm, 10:4）
    output reg o_branch,  // 是否为条件分支指令
    output reg o_jalr,  // 是否为JALR指令
    // 新增CSR相关输出
    output reg o_ecall,  // 是否为ECALL指令
    output reg o_csr_we,  // CSR写使能
    output [11:0] o_csr_addr,  // CSR地址
    output reg [1:0] o_csr_op,  // CSR操作类型(00:无操作, 01:CSRRW, 10:CSRRS)
    output reg o_mret  // 是否为MRET指令
);

  assign o_rs1_addr = i_inst[19:15];
  assign o_rs2_addr = i_inst[24:20];
  assign o_rd_addr  = i_inst[11:7];
  assign o_funct3   = i_inst[14:12];
  assign o_funct7   = i_inst[31:25];
  assign o_opcode   = i_inst[6:0];
  assign o_csr_addr = i_inst[31:20];
  wire [31:0] w_imm_i = {{20{i_inst[31]}}, i_inst[31:20]};
  wire [31:0] w_imm_s = {{20{i_inst[31]}}, i_inst[31:25], i_inst[11:7]};
  wire [31:0] w_imm_b = {{20{i_inst[31]}}, i_inst[7], i_inst[30:25], i_inst[11:8], 1'b0};
  wire [31:0] w_imm_u = {i_inst[31:12], 12'b0};
  wire [31:0] w_imm_j = {{12{i_inst[31]}}, i_inst[19:12], i_inst[20], i_inst[30:21], 1'b0};
  reg  [31:0] r_imm_out;
  always @(*) begin
    case (o_opcode)
      7'b1110011: r_imm_out = 32'b1;
      7'b0010011,
      7'b0000011,
      7'b1100111: r_imm_out = w_imm_i;
      7'b0100011: r_imm_out = w_imm_s;
      7'b1100011: r_imm_out = w_imm_b;
      7'b0010111,
      7'b0110111: r_imm_out = w_imm_u;
      7'b1101111: r_imm_out = w_imm_j;
      7'b0110011: r_imm_out = 32'b0;
      default:    r_imm_out = 32'b1;
    endcase
  end
  assign o_imm = r_imm_out;

  // 默认控制信号（安全状态）
  localparam [3:0] DEFAULT_ALU_OP = `ALU_ADD;
  localparam [1:0] DEFAULT_ALU_SRC1_SEL = `SRC_REG;
  localparam [1:0] DEFAULT_ALU_SRC2_SEL = `SRC_IMM;

  // 主要译码逻辑
  always @(*) begin
    if (i_rst || ~i_inst_valid) begin  // 指令复位或无效时保持安全状态
      o_lsu_wen = 1'b0;
      o_lsu_wmask = 4'b0000;
      o_lsu_rmask = 4'b0000;
      o_reg_wen = 1'b0;
      o_jump = 1'b0;
      o_branch = 1'b0;
      o_jalr = 1'b0;
      o_ecall = 1'b0;
      o_mret = 1'b0;
      o_csr_we = 1'b0;
      o_csr_op = 2'b00;
      o_alu_op = DEFAULT_ALU_OP;
      o_alu_src1_sel = DEFAULT_ALU_SRC1_SEL;
      o_alu_src2_sel = DEFAULT_ALU_SRC2_SEL;
    end else begin
      case (o_opcode)
        7'b0000011: begin  // 加载指令 (LB/LBU/LH/LHU/LW)
          case (o_funct3)
            3'b000: begin  // LB
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b1001;  // 字节有符号扩展: 最高位1表示符号扩展，最低位1表示字节
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b001: begin  // LH
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b1010;  // 半字有符号扩展: 最高位1表示符号扩展，0010表示半字
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b010: begin  // LW
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0100;  // 全字: 0100
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b100: begin  // LBU
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0001;  // 字节无符号扩展: 最高位0表示无符号扩展，最低位1表示字节
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b101: begin  // LHU
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0010;  // 半字无符号扩展: 最高位0表示无符号扩展，0010表示半字
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            default: begin
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0100;  // 默认全字
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;

            end
          endcase
        end

        7'b0010011: begin  // I型算术指令
          case (o_funct3)
            3'b000: begin  // ADDI
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b010: begin  // SLTI
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_SLT;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b011: begin  // SLTIU
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_SLTU;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b100: begin  // XORI
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_XOR;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b110: begin  // ORI
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_OR;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b111: begin  // ANDI
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_AND;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b001: begin  // SLLI
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_SLL;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b101: begin  // SRLI/SRAI
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
              if (i_inst[30] == 1'b0) o_alu_op = `ALU_SRL;  // SRLI
              else o_alu_op = `ALU_SRA;  // SRAI
            end
            default: begin
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;

            end
          endcase
        end

        7'b0010111: begin  // auipc
          o_lsu_wen = 1'b0;
          o_lsu_wmask = 4'b0000;
          o_lsu_rmask = 4'b0000;
          o_reg_wen = 1'b1;
          o_jump = 1'b0;
          o_branch = 1'b0;
          o_jalr = 1'b0;
          o_ecall = 1'b0;
          o_csr_we = 1'b0;
          o_csr_op = 2'b00;
          o_mret = 1'b0;
          o_alu_op = `ALU_ADD;
          o_alu_src1_sel = `SRC_PC;
          o_alu_src2_sel = `SRC_IMM;
        end

        7'b1100111: begin  // jalr
          o_lsu_wen = 1'b0;
          o_lsu_wmask = 4'b0000;
          o_lsu_rmask = 4'b0000;
          o_reg_wen = 1'b1;
          o_jump = 1'b1;
          o_branch = 1'b0;
          o_jalr = 1'b1;
          o_ecall = 1'b0;
          o_csr_we = 1'b0;
          o_csr_op = 2'b00;
          o_mret = 1'b0;
          o_alu_op = `ALU_ADD;
          o_alu_src1_sel = `SRC_REG;
          o_alu_src2_sel = `SRC_IMM;
        end

        7'b1101111: begin  // jal
          o_lsu_wen = 1'b0;
          o_lsu_wmask = 4'b0000;
          o_lsu_rmask = 4'b0000;
          o_reg_wen = 1'b1;
          o_jump = 1'b1;
          o_branch = 1'b0;
          o_jalr = 1'b0;
          o_ecall = 1'b0;
          o_csr_we = 1'b0;
          o_csr_op = 2'b00;
          o_mret = 1'b0;
          o_alu_op = `ALU_ADD;
          o_alu_src1_sel = `SRC_PC;
          o_alu_src2_sel = `SRC_IMM;
        end

        7'b0110111: begin  // lui
          o_lsu_wen = 1'b0;
          o_lsu_wmask = 4'b0000;
          o_lsu_rmask = 4'b0000;
          o_reg_wen = 1'b1;
          o_jump = 1'b0;
          o_branch = 1'b0;
          o_jalr = 1'b0;
          o_ecall = 1'b0;
          o_csr_we = 1'b0;
          o_csr_op = 2'b00;
          o_mret = 1'b0;
          o_alu_op = `ALU_LUI;
          o_alu_src1_sel = `SRC_ZERO;
          o_alu_src2_sel = `SRC_IMM;
        end

        7'b0100011: begin  // S型存储指令 (SB/SH/SW)
          case (o_funct3)
            3'b000: begin  // SB
              o_lsu_wen = 1'b1;
              o_lsu_wmask = 4'b0001;  // 字节基础掩码，LSU会根据地址左移
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b001: begin  // SH
              o_lsu_wen = 1'b1;
              o_lsu_wmask = 4'b0011;  // 半字基础掩码，LSU会根据地址左移
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            3'b010: begin  // SW
              o_lsu_wen = 1'b1;
              o_lsu_wmask = 4'b1111;  // 全字掩码，不需要左移
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;
            end
            default: begin
              o_lsu_wen = 1'b1;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_IMM;

            end
          endcase
        end

        7'b0110011: begin  // R型算术指令
          case (o_funct3)
            3'b000: begin  // ADD/SUB
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
              if (o_funct7 == `FUNCT7_STD) o_alu_op = `ALU_ADD;
              else if (o_funct7 == `FUNCT7_ALT) o_alu_op = `ALU_SUB;
              else begin
                o_alu_op = `ALU_ADD;

              end
            end
            3'b001: begin  // SLL
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
              if (o_funct7 == `FUNCT7_STD) o_alu_op = `ALU_SLL;
              else begin
                o_alu_op = `ALU_SLL;

              end
            end
            3'b010: begin  // SLT
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
              if (o_funct7 == `FUNCT7_STD) o_alu_op = `ALU_SLT;
              else begin
                o_alu_op = `ALU_SLT;

              end
            end
            3'b011: begin  // SLTU
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
              if (o_funct7 == `FUNCT7_STD) o_alu_op = `ALU_SLTU;
              else begin
                o_alu_op = `ALU_SLTU;

              end
            end
            3'b100: begin  // XOR
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
              if (o_funct7 == `FUNCT7_STD) o_alu_op = `ALU_XOR;
              else begin
                o_alu_op = `ALU_XOR;

              end
            end
            3'b101: begin  // SRL/SRA
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
              if (o_funct7 == `FUNCT7_STD) o_alu_op = `ALU_SRL;
              else if (o_funct7 == `FUNCT7_ALT) o_alu_op = `ALU_SRA;
              else begin
                o_alu_op = `ALU_SRL;

              end
            end
            3'b110: begin  // OR
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
              if (o_funct7 == `FUNCT7_STD) o_alu_op = `ALU_OR;
              else begin
                o_alu_op = `ALU_OR;

              end
            end
            3'b111: begin  // AND
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
              if (o_funct7 == `FUNCT7_STD) o_alu_op = `ALU_AND;
              else begin
                o_alu_op = `ALU_AND;

              end
            end
            default: begin
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b1;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;

            end
          endcase
        end

        7'b1100011: begin  // 条件分支指令 (B型)
          case (o_funct3)
            3'b000: begin  // BEQ
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b1;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_SUB;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
            end
            3'b001: begin  // BNE
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b1;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_SUB;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
            end
            3'b100: begin  // BLT
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b1;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_SLT;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
            end
            3'b101: begin  // BGE
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b1;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_SLT;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
            end
            3'b110: begin  // BLTU
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b1;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_SLTU;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
            end
            3'b111: begin  // BGEU
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b1;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_SLTU;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
            end
            default: begin
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b1;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_SUB;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;

            end
          endcase
        end

        7'b1110011: begin  // 系统指令 (ECALL/EBREAK/CSR指令/MRET)
          case (o_funct3)
            3'b000: begin  // ECALL/EBREAK/MRET
              if (i_inst[31:20] == 12'h000) begin  // ECALL
                o_lsu_wen = 1'b0;
                o_lsu_wmask = 4'b0000;
                o_lsu_rmask = 4'b0000;
                o_reg_wen = 1'b0;
                o_jump = 1'b0;
                o_branch = 1'b0;
                o_jalr = 1'b0;
                o_ecall = 1'b1;
                o_csr_we = 1'b0;
                o_csr_op = 2'b00;
                o_mret = 1'b0;
                o_alu_op = `ALU_ADD;
                o_alu_src1_sel = `SRC_REG;
                o_alu_src2_sel = `SRC_REG;
              end else if (i_inst[31:20] == `MRET_INST) begin  // MRET
                o_lsu_wen = 1'b0;
                o_lsu_wmask = 4'b0000;
                o_lsu_rmask = 4'b0000;
                o_reg_wen = 1'b0;
                o_jump = 1'b0;
                o_branch = 1'b0;
                o_jalr = 1'b0;
                o_ecall = 1'b0;
                o_csr_we = 1'b0;
                o_csr_op = 2'b00;
                o_mret = 1'b1;
                o_alu_op = `ALU_ADD;
                o_alu_src1_sel = `SRC_REG;
                o_alu_src2_sel = `SRC_REG;
              end else begin  // 未识别的系统指令
                o_lsu_wen = 1'b0;
                o_lsu_wmask = 4'b0000;
                o_lsu_rmask = 4'b0000;
                o_reg_wen = 1'b0;
                o_jump = 1'b0;
                o_branch = 1'b0;
                o_jalr = 1'b0;
                o_ecall = 1'b0;
                o_csr_we = 1'b0;
                o_csr_op = 2'b00;
                o_mret = 1'b0;
                o_alu_op = `ALU_ADD;
                o_alu_src1_sel = `SRC_REG;
                o_alu_src2_sel = `SRC_REG;

              end
            end
            3'b001: begin  // CSRRW
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = (o_rd_addr != 5'b0) ? 1'b1 : 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b1;
              o_csr_op = 2'b01;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
            end
            3'b010: begin  // CSRRS
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = (o_rd_addr != 5'b0) ? 1'b1 : 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b1;
              o_csr_op = 2'b10;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;
            end
            default: begin
              o_lsu_wen = 1'b0;
              o_lsu_wmask = 4'b0000;
              o_lsu_rmask = 4'b0000;
              o_reg_wen = 1'b0;
              o_jump = 1'b0;
              o_branch = 1'b0;
              o_jalr = 1'b0;
              o_ecall = 1'b0;
              o_csr_we = 1'b0;
              o_csr_op = 2'b00;
              o_mret = 1'b0;
              o_alu_op = `ALU_ADD;
              o_alu_src1_sel = `SRC_REG;
              o_alu_src2_sel = `SRC_REG;

            end
          endcase
        end

        default: begin
          o_lsu_wen = 1'b0;
          o_lsu_wmask = 4'b0000;
          o_lsu_rmask = 4'b0000;
          o_reg_wen = 1'b0;
          o_jump = 1'b0;
          o_branch = 1'b0;
          o_jalr = 1'b0;
          o_ecall = 1'b0;
          o_csr_we = 1'b0;
          o_csr_op = 2'b00;
          o_mret = 1'b0;
          o_alu_op = `ALU_ADD;
          o_alu_src1_sel = `SRC_REG;
          o_alu_src2_sel = `SRC_IMM;

        end
      endcase
    end
  end
endmodule
module ysyx_24090003_EXU (
    // 数据输入
    input [31:0] i_pc,
    input [31:0] i_imm,
    input [31:0] i_rs1_data,
    input [31:0] i_rs2_data,

    // 从IDU接收的控制信号
    input [3:0] i_alu_op,
    input [1:0] i_alu_src1_sel,  // （00:reg_data1, 01:pc, 10:0）
    input [1:0] i_alu_src2_sel,  // （00:reg_data2, 01:imm, 10:4）
    input       i_jump,
    input       i_jalr,
    input       i_branch,
    input [2:0] i_funct3,
    input [6:0] i_funct7,

    input        i_ecall,
    input [31:0] i_csr_rdata,
    input [31:0] i_mtvec,
    input        i_mret,
    input [31:0] i_mepc,

    output [31:0] o_alu_result,  // ALU运算结果
    output [31:0] o_next_pc,     // 下一PC值
    output        o_pc_update,   // PC更新使能信号
    output [31:0] o_csr_wdata    // CSR写入数据
);



  reg [31:0] r_alu_in1, r_alu_in2;
  always @(*) begin
    case (i_alu_src1_sel)
      `SRC_REG:  r_alu_in1 = i_rs1_data;
      `SRC_PC:   r_alu_in1 = i_pc;
      `SRC_ZERO: r_alu_in1 = 32'b0;
      default:   r_alu_in1 = i_rs1_data;
    endcase
  end
  always @(*) begin
    case (i_alu_src2_sel)
      `SRC_REG:  r_alu_in2 = i_rs2_data;
      `SRC_IMM:  r_alu_in2 = i_imm;
      `SRC_FOUR: r_alu_in2 = `CONSTANT_FOUR;
      default:   r_alu_in2 = i_imm;
    endcase
  end
  wire w_alu_zero;
  ysyx_24090003_ALU alu_inst (
      .i_op1   (r_alu_in1),
      .i_op2   (r_alu_in2),
      .i_alu_op(i_alu_op),
      .o_result(o_alu_result),
      .o_zero  (w_alu_zero)
  );

  reg r_branch_taken;
  always @(*) begin
    if (i_branch) begin
      case (i_funct3)
        `BRANCH_BEQ:  r_branch_taken = w_alu_zero;  // BEQ
        `BRANCH_BNE:  r_branch_taken = ~w_alu_zero;  // BNE
        `BRANCH_BLT:  r_branch_taken = o_alu_result[0];  // BLT
        `BRANCH_BGE:  r_branch_taken = ~o_alu_result[0];  // BGE
        `BRANCH_BLTU: r_branch_taken = o_alu_result[0];  // BLTU
        `BRANCH_BGEU: r_branch_taken = ~o_alu_result[0];  // BGEU
        default:      r_branch_taken = 1'b0;
      endcase
    end else begin
      r_branch_taken = 1'b0;
    end
  end

  // PC更新使能信号 - 无条件跳转、条件分支成功、ECALL异常或MRET返回时更新PC
  assign o_pc_update = i_jump || (i_branch && r_branch_taken) || i_ecall || i_mret;

  assign o_csr_wdata = i_rs1_data;









  reg [31:0] r_pc_alu_in1, r_pc_alu_in2;
  wire [31:0] w_pc_alu_result;
  always @(*) begin
    if (i_ecall) begin
      // ECALL异常: 跳转到mtvec地址
      r_pc_alu_in1 = i_mtvec;
      r_pc_alu_in2 = 32'h0;
    end else if (i_mret) begin
      // MRET返回: 跳转到mepc地址
      r_pc_alu_in1 = i_mepc;
      r_pc_alu_in2 = 32'h0;
    end else if (i_jump) begin
      if (i_jalr) begin
        // JALR: PC = (rs1 + imm) & ~1
        r_pc_alu_in1 = i_rs1_data;
        r_pc_alu_in2 = i_imm;
      end else begin
        // JAL: PC = pc + imm
        r_pc_alu_in1 = i_pc;
        r_pc_alu_in2 = i_imm;
      end
    end else if (i_branch && r_branch_taken) begin
      // 条件分支跳转: PC = pc + imm
      r_pc_alu_in1 = i_pc;
      r_pc_alu_in2 = i_imm;
    end else begin
      // 非跳转指令，输入设为0(实际不使用)
      r_pc_alu_in1 = 32'h0;
      r_pc_alu_in2 = 32'h0;
    end
  end
  ysyx_24090003_ALU pc_alu_inst (
      .i_op1   (r_pc_alu_in1),
      .i_op2   (r_pc_alu_in2),
      .i_alu_op(`ALU_ADD),
      .o_result(w_pc_alu_result),
      .o_zero  ()
  );
  reg [31:0] r_next_pc_pre;
  always @(*) begin
    if (i_ecall || i_mret) begin
      r_next_pc_pre = w_pc_alu_result;
    end else if (i_jalr) begin
      r_next_pc_pre = w_pc_alu_result & `JALR_MASK;
    end else begin
      r_next_pc_pre = w_pc_alu_result;
    end
  end

  assign o_next_pc = r_next_pc_pre;
endmodule
module ysyx_24090003_LSU (
    input i_clk,  // 时钟信号
    input i_rst,  // 复位信号
    input [31:0] i_lsu_addr,  // 内存地址
    input [31:0] i_lsu_wdata,  // 写入数据
    input i_lsu_wen,  // 内存写使能信号（1：写，0：读）
    input [31:0] i_lsu_rdata,  // 从外部传入的读取数据
    input i_lsu_respValid,  // 从computer传入的响应有效信号
    input [3:0] i_lsu_wmask,  // 4位写掩码（0001:byte, 0010:halfword, 0100:word）
    input [3:0]  i_lsu_rmask,   // 4位读掩码（0001:byte无符号, 0010:halfword无符号, 0100:word, 1001:byte有符号, 1010:halfword有符号）

    // 来自IFU的阶段信号
    input       i_ifu_inst_valid,  // IFU指令有效信号
    input [3:0] i_ifu_state,       // IFU状态信号

    // 输出到computer的信号
    output [31:0] o_lsu_addr,      // 输出到computer的地址
    output [31:0] o_lsu_wdata,     // 输出到computer的写数据
    output        o_lsu_wen,       // 输出到computer的写使能
    output        o_lsu_reqValid,  // 输出到computer的内存请求有效信号
    output [ 3:0] o_lsu_wmask,     // 4位写掩码传递给外部
    output [ 1:0] o_lsu_size,      // 2位数据位宽信号（00:1字节, 01:2字节, 10:4字节）
    output [31:0] o_lsu_rdata,     // 读出数据（传给其他模块）
    output        o_load_inst,     // Load指令信号（用于激活IFU进入load_wait状态）
    output        o_store_inst,    // Store指令信号（用于激活IFU进入store_wait状态）
    output        o_load_reg_wen,  // Load写回寄存器使能（用于WBU写回控制）
    output        o_load_done,     // Load操作完成信号（仅用于load指令）
    output        o_store_done     // Store操作完成信号（仅用于store指令）
);
  localparam STATE_IDLE = 2'b00;  // 空闲状态
  localparam STATE_LOAD_WAIT = 2'b01;  // LOAD等待响应阶段并处理数据
  localparam STATE_STORE_WAIT = 2'b10;  // STORE等待响应阶段

  reg [1:0] r_state, next_state;  // 状态寄存器和下一状态（2位）
  reg  [31:0] r_lsu_addr;  // 地址寄存器
  reg  [31:0] r_lsu_wdata;  // 写数据寄存器
  reg  [ 3:0] r_lsu_wmask;  // 写掩码寄存器
  reg  [ 3:0] r_lsu_rmask;  // 读掩码寄存器
  reg  [31:0] r_lsu_rdata;  // 读数据寄存器
  reg         r_load_done;  // Load完成信号寄存器
  reg         r_store_done;  // Store完成信号寄存器

  // 输出寄存器
  reg  [31:0] r_o_lsu_addr;  // 输出地址寄存器
  reg  [31:0] r_o_lsu_wdata;  // 输出写数据寄存器
  reg         r_o_lsu_wen;  // 输出写使能寄存器
  reg         r_o_lsu_reqValid;  // 输出请求有效寄存器
  reg  [ 3:0] r_o_lsu_wmask;  // 输出写掩码寄存器
  reg  [ 1:0] r_o_lsu_size;  // 输出数据位宽寄存器
  // load_reg_wen信号 - 用于WBU写回控制
  reg         r_load_reg_wen;
  // 根据地址和基础掩码计算实际的写掩码
  wire [ 3:0] w_actual_wmask;
  reg  [ 3:0] r_actual_wmask;
  // load_inst信号（load指令）- 用于激活IFU进入load_wait状态
  // 通过读掩码判断是否为load指令
  wire        w_load_inst;
  // store_inst信号（store指令）
  wire        w_store_inst;
  // 根据地址和基础掩码计算实际的写数据
  wire [31:0] w_actual_wdata;
  reg  [31:0] r_actual_wdata;
  // 组合逻辑：根据掩码计算size信号
  reg [1:0] w_lsu_size;
  assign w_load_inst = (|i_lsu_rmask) && (~i_lsu_wen) && i_ifu_inst_valid;
  assign w_store_inst = (|w_actual_wmask) && i_lsu_wen && i_ifu_inst_valid;
  always @(*) begin
    case (i_lsu_wmask)
      4'b0001: begin  // SB: 字节掩码根据地址偏移
        case (i_lsu_addr[1:0])
          2'b00: r_actual_wmask = 4'b0001;  // 字节0
          2'b01: r_actual_wmask = 4'b0010;  // 字节1
          2'b10: r_actual_wmask = 4'b0100;  // 字节2
          2'b11: r_actual_wmask = 4'b1000;  // 字节3
        endcase
      end
      4'b0011: begin  // SH: 半字掩码根据地址[1]位偏移
        case (i_lsu_addr[1])
          1'b0: r_actual_wmask = 4'b0011;  // 半字0 (字节0,1)
          1'b1: r_actual_wmask = 4'b1100;  // 半字1 (字节2,3)
        endcase
      end
      4'b1111: r_actual_wmask = 4'b1111;  // SW: 全字掩码不需要偏移
      default: r_actual_wmask = 4'b0000;
    endcase
  end
  assign w_actual_wmask = r_actual_wmask;
  always @(*) begin
    case (i_lsu_wmask)
      4'b0001: begin  // SB: 字节数据根据地址偏移
        case (i_lsu_addr[1:0])
          2'b00: r_actual_wdata = {24'b0, i_lsu_wdata[7:0]};  // 字节0位置
          2'b01: r_actual_wdata = {16'b0, i_lsu_wdata[7:0], 8'b0};  // 字节1位置
          2'b10: r_actual_wdata = {8'b0, i_lsu_wdata[7:0], 16'b0};  // 字节2位置
          2'b11: r_actual_wdata = {i_lsu_wdata[7:0], 24'b0};  // 字节3位置
        endcase
      end
      4'b0011: begin  // SH: 半字数据根据地址[1]位偏移
        case (i_lsu_addr[1])
          1'b0: r_actual_wdata = {16'b0, i_lsu_wdata[15:0]};  // 半字0位置
          1'b1: r_actual_wdata = {i_lsu_wdata[15:0], 16'b0};  // 半字1位置
        endcase
      end
      4'b1111: r_actual_wdata = i_lsu_wdata;  // SW: 全字数据不需要偏移
      default: r_actual_wdata = 32'b0;
    endcase
  end
  assign w_actual_wdata = r_actual_wdata;
  always @(*) begin
    case ({
      i_lsu_rmask, i_lsu_wmask
    })
      // 读操作根据读掩码计算
      {
        4'b0001, 4'b0000
      },  // LBU - 字节无符号
      {
        4'b1001, 4'b0000
      } :  // LB - 字节有符号
      w_lsu_size = 2'b00;  // 1字节
      {
        4'b0010, 4'b0000
      },  // LHU - 半字无符号
      {
        4'b1010, 4'b0000
      } :  // LH - 半字有符号
      w_lsu_size = 2'b01;  // 2字节
      {
        4'b0100, 4'b0000
      } :  // LW - 全字
      w_lsu_size = 2'b10;  // 4字节
      // 写操作根据基础写掩码计算
      {
        4'b0000, 4'b0001
      } :  // SB - 字节基础掩码
      w_lsu_size = 2'b00;  // 1字节
      {
        4'b0000, 4'b0011
      } :  // SH - 半字基础掩码  
      w_lsu_size = 2'b01;  // 2字节
      {
        4'b0000, 4'b1111
      } :  // SW - 全字掩码
      w_lsu_size = 2'b10;  // 4字节
      default: w_lsu_size = 2'b10;  // 默认为4字节
    endcase
  end
  // 第一段：组合逻辑 - 状态转移逻辑
  always @(*) begin
    case (r_state)
      STATE_IDLE: begin
        if (w_load_inst) begin
          next_state = STATE_LOAD_WAIT;  // load指令进入LOAD_WAIT状态
        end else if (w_store_inst) begin
          next_state = STATE_STORE_WAIT;  // store指令进入STORE_WAIT状态
        end else begin
          next_state = STATE_IDLE;  // 无请求保持在IDLE状态
        end
      end

      STATE_LOAD_WAIT: begin
        if (i_lsu_respValid) begin
          next_state = STATE_IDLE;
        end else begin
          next_state = STATE_LOAD_WAIT;  // 继续等待
        end
      end

      STATE_STORE_WAIT: begin
        if (i_lsu_respValid) begin
          next_state = STATE_IDLE;
        end else begin
          next_state = STATE_STORE_WAIT;  // 继续等待
        end
      end

      default: begin
        next_state = STATE_IDLE;
      end
    endcase
  end

  // 第二段：时序逻辑 - 状态更新和寄存器更新
  always @(posedge i_clk) begin
    if (i_rst) begin
      r_state <= STATE_IDLE;
      r_lsu_addr <= 32'b0;
      r_lsu_wdata <= 32'b0;
      r_lsu_wmask <= 4'b0;
      r_lsu_rmask <= 4'b0;
      r_lsu_rdata <= 32'b0;
      r_load_done <= 1'b0;
      r_store_done <= 1'b0;
      r_load_reg_wen <= 1'b0;

      // 输出寄存器复位
      r_o_lsu_addr <= 32'b0;
      r_o_lsu_wdata <= 32'b0;
      r_o_lsu_wen <= 1'b0;
      r_o_lsu_reqValid <= 1'b0;
      r_o_lsu_wmask <= 4'b0;
      r_o_lsu_size <= 2'b0;
    end else begin
      // 状态更新
      r_state <= next_state;

      // 根据当前状态和输入更新寄存器
      case (r_state)
        STATE_IDLE: begin
          // 处理load和store请求
          if ((|i_lsu_rmask || |i_lsu_wmask) && i_ifu_inst_valid) begin
            // 锁存输入信号
            r_lsu_addr <= i_lsu_addr;
            r_lsu_wdata <= i_lsu_wdata;
            r_lsu_wmask <= i_lsu_wmask;
            r_lsu_rmask <= i_lsu_rmask;

            // 输出请求信号（load和store都需要）
            r_o_lsu_addr <= i_lsu_addr;
            r_o_lsu_reqValid <= 1'b1;
            r_o_lsu_wmask <= (i_lsu_wen) ? w_actual_wmask : i_lsu_wmask;  // 写操作使用实际掩码，读操作使用原掩码
            r_o_lsu_size <= w_lsu_size;

            // 确保写数据和写使能同时出现和同时消失
            if (i_lsu_wen) begin
              // 写操作：同时设置写数据和写使能，使用对齐后的数据
              r_o_lsu_wdata <= w_actual_wdata;
              r_o_lsu_wen   <= 1'b1;
            end else begin
              // 读操作：同时清除写数据和写使能
              r_o_lsu_wdata <= 32'b0;
              r_o_lsu_wen   <= 1'b0;
            end
          end else begin
            // 清除输出信号 - 确保写数据和写使能同时消失
            r_o_lsu_wdata <= 32'b0;
            r_o_lsu_wen <= 1'b0;
            r_o_lsu_reqValid <= 1'b0;
          end
          // IDLE状态清除控制信号
          r_load_done <= 1'b0;
          r_store_done <= 1'b0;
          r_load_reg_wen <= 1'b0;
        end
        STATE_LOAD_WAIT: begin
          // LOAD等待阶段：等待响应
          if (i_lsu_respValid) begin
            // 收到load响应，处理返回的数据，根据地址偏移和读掩码进行提取和符号扩展
            case (r_lsu_rmask)
              4'b0001: begin  // 字节无符号扩展 (lbu)
                case (r_lsu_addr[1:0])
                  2'b00: r_lsu_rdata <= {24'b0, i_lsu_rdata[7:0]};
                  2'b01: r_lsu_rdata <= {24'b0, i_lsu_rdata[15:8]};
                  2'b10: r_lsu_rdata <= {24'b0, i_lsu_rdata[23:16]};
                  2'b11: r_lsu_rdata <= {24'b0, i_lsu_rdata[31:24]};
                endcase
              end
              4'b1001: begin  // 字节有符号扩展 (lb)
                case (r_lsu_addr[1:0])
                  2'b00: r_lsu_rdata <= {{24{i_lsu_rdata[7]}}, i_lsu_rdata[7:0]};
                  2'b01: r_lsu_rdata <= {{24{i_lsu_rdata[15]}}, i_lsu_rdata[15:8]};
                  2'b10: r_lsu_rdata <= {{24{i_lsu_rdata[23]}}, i_lsu_rdata[23:16]};
                  2'b11: r_lsu_rdata <= {{24{i_lsu_rdata[31]}}, i_lsu_rdata[31:24]};
                endcase
              end
              4'b0010: begin  // 半字无符号扩展 (lhu)
                case (r_lsu_addr[1])
                  1'b0: r_lsu_rdata <= {16'b0, i_lsu_rdata[15:0]};
                  1'b1: r_lsu_rdata <= {16'b0, i_lsu_rdata[31:16]};
                endcase
              end
              4'b1010: begin  // 半字有符号扩展 (lh)
                case (r_lsu_addr[1])
                  1'b0: r_lsu_rdata <= {{16{i_lsu_rdata[15]}}, i_lsu_rdata[15:0]};
                  1'b1: r_lsu_rdata <= {{16{i_lsu_rdata[31]}}, i_lsu_rdata[31:16]};
                endcase
              end
              4'b0100: r_lsu_rdata <= i_lsu_rdata;  // 字访问 (lw)
              default: r_lsu_rdata <= i_lsu_rdata;  // 默认字访问
            endcase
          end else begin
            r_lsu_rdata <= 32'b0;  // 未收到load响应时清空数据
          end
          // 确保写数据和写使能同时清除
          r_o_lsu_wdata <= 32'b0;
          r_o_lsu_wen <= 1'b0;
          r_o_lsu_reqValid <= 1'b0;
          r_load_done <= i_lsu_respValid;     // Load操作完成信号
          r_load_reg_wen <= i_lsu_respValid;  // 写回寄存器使能信号
        end
        STATE_STORE_WAIT: begin
          // STORE等待阶段：等待响应
          r_o_lsu_wdata <= 32'b0;
          r_o_lsu_wen <= 1'b0;
          r_o_lsu_reqValid <= 1'b0;
          r_store_done <= i_lsu_respValid;  // Store操作完成信号
        end
        default: begin
          r_o_lsu_wdata <= 32'b0;
          r_o_lsu_wen <= 1'b0;
          r_o_lsu_reqValid <= 1'b0;
          r_load_done <= 1'b0;
          r_store_done <= 1'b0;
          r_load_reg_wen <= 1'b0;
        end
      endcase
    end
  end

  // 输出赋值 - 通过输出寄存器驱动
  assign o_lsu_addr = r_o_lsu_addr;
  assign o_lsu_wdata = r_o_lsu_wdata;
  assign o_lsu_wen = r_o_lsu_wen;
  assign o_lsu_reqValid = r_o_lsu_reqValid;
  assign o_lsu_wmask = r_o_lsu_wmask;
  assign o_lsu_size = r_o_lsu_size;
  assign o_lsu_rdata = r_lsu_rdata;
  assign o_load_inst = w_load_inst;     // load指令信号，用于激活IFU进入load_wait状态
  assign o_store_inst = w_store_inst;   // store指令信号，用于激活IFU进入store_wait状态
  assign o_load_reg_wen = r_load_reg_wen; // load写回寄存器使能，用于WBU写回控制
  assign o_load_done = r_load_done;     // Load操作完成信号（仅用于load指令）
  assign o_store_done = r_store_done;   // Store操作完成信号（仅用于store指令）
endmodule
module ysyx_24090003_WBU (
    input        i_clk,          // 时钟
    input        i_rst,          // 复位信号，高电平有效
    input [ 4:0] i_rd_addr,      // 目标寄存器地址
    input [ 4:0] i_rs1_addr,     // 源寄存器1地址
    input [ 4:0] i_rs2_addr,     // 源寄存器2地址
    input [31:0] i_alu_result,   // ALU结果
    input [31:0] i_mem_rdata,    // 内存读取数据
    input [31:0] i_pc,           // 当前程序计数器
    input        i_reg_wen,      // 寄存器写使能
    input        i_jump,         // 跳转信号
    input        i_load_reg_wen, // Load写回寄存器使能信号

    // 新增CSR相关输入
    input        i_csr_we,     // CSR写使能
    input [11:0] i_csr_addr,   // CSR地址
    input [ 1:0] i_csr_op,     // CSR操作类型
    input [31:0] i_csr_wdata,  // CSR写入数据
    input        i_ecall,      // ECALL异常信号
    input        i_mret,       // MRET指令信号

    output [31:0] o_rs1_data,   // 读出寄存器1数据
    output [31:0] o_rs2_data,   // 读出寄存器2数据
    output [31:0] o_rd_wdata,   // 写入寄存器的数据
    output [31:0] o_csr_rdata,  // CSR读取数据
    output [31:0] o_mtvec,      // mtvec寄存器值
    output [31:0] o_mepc        // mepc寄存器值
);
  // 二层判断确定写回数据：
  // 第一层：指令类型判断（jump/csr/普通指令）
  // 第二层：普通指令时判断ALU数据还是内存数据（通过load_valid）
  reg [31:0] r_rd_wdata;
  // 寄存器写使能控制：
  // 对于load指令：只有load_reg_wen为高时才写回
  // 对于其他指令：正常写回
  wire w_reg_wen_final;
  always @(*) begin
    if (i_jump) begin
      r_rd_wdata = i_pc + 4;  // JAL/JALR: PC+4写回rd
    end else if (i_csr_we) begin
      // CSR指令：根据操作类型决定写回数据
      case (i_csr_op)
        2'b01:   r_rd_wdata = o_csr_rdata;  // CSRRW: 写回CSR旧值
        2'b10:   r_rd_wdata = o_csr_rdata;  // CSRRS: 写回CSR旧值
        default: r_rd_wdata = o_csr_rdata;
      endcase
    end else begin
      // 普通指令的二层判断：写ALU数据还是内存数据
      if (i_load_reg_wen) begin
        r_rd_wdata = i_mem_rdata;  // 写内存数据
      end else begin
        r_rd_wdata = i_alu_result;  // 写ALU数据
      end
    end
  end
  assign o_rd_wdata = r_rd_wdata;
  assign w_reg_wen_final = i_reg_wen | i_load_reg_wen;
  // 实例化寄存器文件
  ysyx_24090003_RegFile regfile (
      .i_clk(i_clk),
      .i_rst(i_rst),
      .i_rs1_addr(i_rs1_addr),
      .i_rs2_addr(i_rs2_addr),
      .i_rd_addr(i_rd_addr),
      .i_rd_wdata(r_rd_wdata),
      .i_reg_wen(w_reg_wen_final),

      // CSR相关接口
      .i_csr_we(i_csr_we),
      .i_csr_addr(i_csr_addr),
      .i_csr_op(i_csr_op),
      .i_csr_wdata(i_csr_wdata),
      .i_ecall(i_ecall),
      .i_pc(i_pc),
      .i_mret(i_mret),

      .o_rs1_data(o_rs1_data),
      .o_rs2_data(o_rs2_data),
      .o_csr_rdata(o_csr_rdata),
      .o_mtvec(o_mtvec),
      .o_mepc(o_mepc)
  );
endmodule
module ysyx_24090003_RegFile (
    input wire        i_clk,
    input wire        i_rst,
    input wire [ 4:0] i_rs1_addr,
    input wire [ 4:0] i_rs2_addr,
    input wire [ 4:0] i_rd_addr,
    input wire [31:0] i_rd_wdata,
    input wire        i_reg_wen,
    input wire        i_csr_we,
    input wire [11:0] i_csr_addr,
    input wire [ 1:0] i_csr_op,
    input wire [31:0] i_csr_wdata,
    input wire        i_ecall,
    input wire [31:0] i_pc,
    input wire        i_mret,

    output wire [31:0] o_rs1_data,
    output wire [31:0] o_rs2_data,
    output wire [31:0] o_csr_rdata,
    output wire [31:0] o_mtvec,
    output wire [31:0] o_mepc
);
  //正常16个但是译码就要取低四位，为了简便直接取32个后面有问题再说


  reg [31:0] r_gpr                             [31:0];
  reg [31:0] r_mvendorid;  // 厂商ID寄存器
  reg [31:0] r_marchid;  // 架构ID寄存器  
  reg [31:0] r_mstatus;
  reg [31:0] r_mtvec;
  reg [31:0] r_mepc;
  reg [31:0] r_mcause;
  reg [63:0] r_mcycle;  // 64位周期计数器
  reg [31:0] r_csr_rdata;
  assign o_rs1_data = (i_rs1_addr == 5'b0) ? 32'b0 : r_gpr[i_rs1_addr];
  assign o_rs2_data = (i_rs2_addr == 5'b0) ? 32'b0 : r_gpr[i_rs2_addr];

  always @(*) begin
    case (i_csr_addr)
      `CSR_MVENDORID: r_csr_rdata = r_mvendorid;
      `CSR_MARCHID:   r_csr_rdata = r_marchid;
      `CSR_MSTATUS:   r_csr_rdata = r_mstatus;
      `CSR_MTVEC:     r_csr_rdata = r_mtvec;
      `CSR_MEPC:      r_csr_rdata = r_mepc;
      `CSR_MCAUSE:    r_csr_rdata = r_mcause;
      `CSR_MCYCLE:    r_csr_rdata = r_mcycle[31:0];  // mcycle低32位
      `CSR_MCYCLEH:   r_csr_rdata = r_mcycle[63:32];  // mcycle高32位
      default:        r_csr_rdata = 32'h0;
    endcase
  end
  assign o_csr_rdata = r_csr_rdata;
  assign o_mtvec = r_mtvec;
  assign o_mepc = r_mepc;
  // 寄存器写入逻辑
  integer i;
  always @(posedge i_clk) begin
    if (i_rst) begin
      for (i = 0; i < 32; i = i + 1) begin
        r_gpr[i] <= 32'b0;
      end
      r_mvendorid <= `MVENDORID_VALUE;  // "ysyx"的ASCII码
      r_marchid   <= `MARCHID_VALUE;  // 学号数字部分
      r_mstatus   <= 32'h1800;
      r_mtvec     <= 32'h0;
      r_mepc      <= 32'h0;
      r_mcause    <= 32'h0;
      r_mcycle    <= 64'h0;
    end else begin
      // 每个时钟周期递增mcycle计数器（除非正在写入mcycle/mcycleh）
      if (!(i_csr_we && (i_csr_addr == `CSR_MCYCLE || i_csr_addr == `CSR_MCYCLEH))) begin
        r_mcycle <= r_mcycle + 64'h1;
      end

      if (i_ecall) begin
        r_mepc   <= i_pc;
        r_mcause <= `ECALL_M_MODE;
      end  // CSR写入
      else if (i_csr_we) begin
        case (i_csr_addr)
          `CSR_MVENDORID: begin
            // mvendorid是只读寄存器，忽略写入操作
          end
          `CSR_MARCHID: begin
            // marchid是只读寄存器，忽略写入操作
          end
          `CSR_MSTATUS: begin
            case (i_csr_op)
              2'b01:   r_mstatus <= i_csr_wdata;  // CSRRW: 直接写入rs1值
              2'b10: begin  // CSRRS: 只有rs1不为x0时才写入
                if (i_rs1_addr != 5'b0) begin
                  r_mstatus <= r_mstatus | i_csr_wdata;
                end
              end
              default: r_mstatus <= i_csr_wdata;
            endcase
          end
          `CSR_MTVEC: begin
            case (i_csr_op)
              2'b01:   r_mtvec <= i_csr_wdata;
              2'b10: begin  // CSRRS: 只有rs1不为x0时才写入
                if (i_rs1_addr != 5'b0) begin
                  r_mtvec <= r_mtvec | i_csr_wdata;
                end
              end
              default: r_mtvec <= i_csr_wdata;
            endcase
          end
          `CSR_MEPC: begin
            case (i_csr_op)
              2'b01:   r_mepc <= i_csr_wdata;
              2'b10: begin  // CSRRS: 只有rs1不为x0时才写入
                if (i_rs1_addr != 5'b0) begin
                  r_mepc <= r_mepc | i_csr_wdata;
                end
              end
              default: r_mepc <= i_csr_wdata;
            endcase
          end
          `CSR_MCAUSE: begin
            case (i_csr_op)
              2'b01:   r_mcause <= i_csr_wdata;
              2'b10: begin  // CSRRS: 只有rs1不为x0时才写入
                if (i_rs1_addr != 5'b0) begin
                  r_mcause <= r_mcause | i_csr_wdata;
                end
              end
              default: r_mcause <= i_csr_wdata;
            endcase
          end
          `CSR_MCYCLE: begin
            case (i_csr_op)
              2'b01:   r_mcycle[31:0] <= i_csr_wdata;  // CSRRW: 直接写入rs1值
              2'b10: begin  // CSRRS: 只有rs1不为x0时才写入
                if (i_rs1_addr != 5'b0) begin
                  r_mcycle[31:0] <= r_mcycle[31:0] | i_csr_wdata;
                end
              end
              default: r_mcycle[31:0] <= i_csr_wdata;
            endcase
          end
          `CSR_MCYCLEH: begin
            case (i_csr_op)
              2'b01:   r_mcycle[63:32] <= i_csr_wdata;  // CSRRW: 直接写入rs1值
              2'b10: begin  // CSRRS: 只有rs1不为x0时才写入
                if (i_rs1_addr != 5'b0) begin
                  r_mcycle[63:32] <= r_mcycle[63:32] | i_csr_wdata;
                end
              end
              default: r_mcycle[63:32] <= i_csr_wdata;
            endcase
          end
          default: begin
          end
        endcase
      end

      if (i_reg_wen && (i_rd_addr != 5'b0)) begin
        r_gpr[i_rd_addr] <= i_rd_wdata;
      end
      r_gpr[0] <= 32'b0;
    end
  end
endmodule
module ysyx_24090003_ALU (
    input      [31:0] i_op1,
    input      [31:0] i_op2,
    input      [ 3:0] i_alu_op,
    output reg [31:0] o_result,
    output            o_zero
);
  wire [31:0] w_add_result;
  wire [31:0] w_sub_result;
  wire        w_overflow;
  wire        w_carry_out;
  wire [31:0] w_sll_result;
  wire [31:0] w_srl_result;
  wire [31:0] w_sra_result;
  wire        w_slt_result;
  wire        w_sltu_result;
  adder_32bit adder (
      .a(i_op1),
      .b(i_op2),
      .cin(1'b0),
      .sum(w_add_result),
      .cout(w_carry_out)
  );
  wire [31:0] w_op2_not = ~i_op2;
  wire [31:0] w_op2_not_plus_1;
  adder_32bit adder_sub (
      .a   (i_op1),
      .b   (w_op2_not),
      .cin (1'b1),
      .sum (w_sub_result),
      .cout()
  );
  // 有符号和无符号比较
  assign w_slt_result  = (i_op1[31] != i_op2[31]) ? i_op1[31] : w_sub_result[31];
  assign w_sltu_result = (i_op1 < i_op2) ? 1'b1 : 1'b0;

  // 左移实现 (桶形移位器)
  wire [31:0] w_sll_level1 = i_op2[0] ? {i_op1[30:0], 1'b0} : i_op1;
  wire [31:0] w_sll_level2 = i_op2[1] ? {w_sll_level1[29:0], 2'b00} : w_sll_level1;
  wire [31:0] w_sll_level3 = i_op2[2] ? {w_sll_level2[27:0], 4'b0000} : w_sll_level2;
  wire [31:0] w_sll_level4 = i_op2[3] ? {w_sll_level3[23:0], 8'b00000000} : w_sll_level3;
  wire [31:0] w_sll_level5 = i_op2[4] ? {w_sll_level4[15:0], 16'b0000000000000000} : w_sll_level4;
  assign w_sll_result = w_sll_level5;

  // 逻辑右移实现
  wire [31:0] w_srl_level1 = i_op2[0] ? {1'b0, i_op1[31:1]} : i_op1;
  wire [31:0] w_srl_level2 = i_op2[1] ? {2'b00, w_srl_level1[31:2]} : w_srl_level1;
  wire [31:0] w_srl_level3 = i_op2[2] ? {4'b0000, w_srl_level2[31:4]} : w_srl_level2;
  wire [31:0] w_srl_level4 = i_op2[3] ? {8'b00000000, w_srl_level3[31:8]} : w_srl_level3;
  wire [31:0] w_srl_level5 = i_op2[4] ? {16'b0000000000000000, w_srl_level4[31:16]} : w_srl_level4;
  assign w_srl_result = w_srl_level5;

  // 算术右移实现
  wire w_sign_bit = i_op1[31];
  wire [31:0] w_sra_level1 = i_op2[0] ? {w_sign_bit, i_op1[31:1]} : i_op1;
  wire [31:0] w_sra_level2 = i_op2[1] ? {{2{w_sign_bit}}, w_sra_level1[31:2]} : w_sra_level1;
  wire [31:0] w_sra_level3 = i_op2[2] ? {{4{w_sign_bit}}, w_sra_level2[31:4]} : w_sra_level2;
  wire [31:0] w_sra_level4 = i_op2[3] ? {{8{w_sign_bit}}, w_sra_level3[31:8]} : w_sra_level3;
  wire [31:0] w_sra_level5 = i_op2[4] ? {{16{w_sign_bit}}, w_sra_level4[31:16]} : w_sra_level4;
  assign w_sra_result = w_sra_level5;

  // ALU结果选择
  always @(*) begin
    case (i_alu_op)
      `ALU_ADD:  o_result = w_add_result;
      `ALU_SUB:  o_result = w_sub_result;
      `ALU_AND:  o_result = i_op1 & i_op2;
      `ALU_OR:   o_result = i_op1 | i_op2;
      `ALU_XOR:  o_result = i_op1 ^ i_op2;
      `ALU_SLL:  o_result = w_sll_result;
      `ALU_SRL:  o_result = w_srl_result;
      `ALU_SRA:  o_result = w_sra_result;
      `ALU_SLT:  o_result = {31'b0, w_slt_result};
      `ALU_SLTU: o_result = {31'b0, w_sltu_result};
      `ALU_LUI:  o_result = i_op2;
      default:   o_result = w_add_result;
    endcase
  end
  assign o_zero = (o_result == 32'b0);

endmodule
module adder_32bit (
    input [31:0] a,
    input [31:0] b,
    input cin,
    output [31:0] sum,
    output cout
);
  wire [32:0] carry;
  assign carry[0] = cin;

  // 串联32个全加器
  genvar i;
  generate
    for (i = 0; i < 32; i = i + 1) begin : gen_adders
      full_adder fa (
          .a(a[i]),
          .b(b[i]),
          .cin(carry[i]),
          .sum(sum[i]),
          .cout(carry[i+1])
      );
    end
  endgenerate

  assign cout = carry[32];
endmodule

// 全加器模块 - 使用半加器和与非门实现
module full_adder (
    input  a,
    input  b,
    input  cin,
    output sum,
    output cout
);
  wire sum1, carry1;
  wire sum2, carry2;

  // 第一个半加器处理a和b
  half_adder ha1 (
      .a(a),
      .b(b),
      .sum(sum1),
      .carry(carry1)
  );

  // 第二个半加器处理sum1和cin
  half_adder ha2 (
      .a(sum1),
      .b(cin),
      .sum(sum),
      .carry(carry2)
  );

  // 最终进位 = 第一个半加器的进位 OR 第二个半加器的进位
  // 用与非门实现OR: ~(~x & ~y) = x | y
  wire not_carry1 = ~carry1;
  wire not_carry2 = ~carry2;
  assign cout = ~(not_carry1 & not_carry2);
endmodule

// 32位加法器模块 - 使用全加器实现

module half_adder (
    input  a,
    input  b,
    output sum,
    output carry
);
  // 使用与非门实现异或(XOR)和与(AND)
  wire nand_ab;
  wire nand_a_nand_ab;
  wire nand_b_nand_ab;

  assign nand_ab = ~(a & b);  // NAND(a, b)
  assign nand_a_nand_ab = ~(a & nand_ab);  // NAND(a, NAND(a, b))
  assign nand_b_nand_ab = ~(b & nand_ab);  // NAND(b, NAND(a, b))
  assign sum = ~(nand_a_nand_ab & nand_b_nand_ab);  // NAND(NAND(a, NAND(a,b)), NAND(b, NAND(a,b)))
  assign carry = ~nand_ab;  // NOT(NAND(a, b)) = AND(a, b)
endmodule
