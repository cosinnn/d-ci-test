module ysyx_25080202_CSR(
    input clk,
    input rst,
    input I_csrrs,
    input I_csrrw,                    //判断指令是不是 csrrw
    input [11:0] csr_addr,            // SR 地址 = inst[31:20]
    input [31:0] csr_wdata,           //要写进CSR的数据
    output reg [31:0] csr_rdata      //从CSR寄存器读到的数据

);

    reg [63:0] mcycle;
 

    localparam [31:0] MVENDORID = 32'h79737978;  // "ysyx"
    localparam [31:0] MARCHID   = 32'h017EB18A;  // 学号部分
    //
    always @(*) begin
        //csr_rdata = 32'b0;
        if(I_csrrs||I_csrrw) begin
        case (csr_addr)
            12'hB00: csr_rdata = mcycle[31:0];
            12'hB80: csr_rdata = mcycle[63:32];
            12'hF11: csr_rdata = MVENDORID;
            12'hF12: csr_rdata = MARCHID;
            default: csr_rdata = 32'b0;
        endcase
        end else begin
           csr_rdata = 32'b0;
        end
    end

    // ========= 时序逻辑：写 =========
    always @(posedge clk) begin
        if (rst) begin
            mcycle <= 64'b0;
        end else begin
            mcycle <= mcycle + 1;

            if (I_csrrw) begin
                case (csr_addr)
                    12'hB00: mcycle[31:0] <= csr_wdata;
                    12'hB80: mcycle[63:32] <= csr_wdata;
                    12'hF11, 12'hF12: ;  // 只读 CSR，不允许写
                    default: ;
                endcase
            end else if (I_csrrs && csr_wdata != 32'b0) begin
                case (csr_addr)
                    12'hB00: mcycle[31:0] <= mcycle[31:0] | csr_wdata;
                    12'hB80: mcycle[63:32] <= mcycle[63:32] | csr_wdata;
                    12'hF11, 12'hF12: ;  // 只读 CSR
                    default: ;
                endcase
            end
        end
    end

endmodule
module ysyx_25080202_EXU(
    input R_TYPE,
    input I_TYPE,
    input S_TYPE,
    input B_TYPE,
    input J_TYPE,
    input U_TYPE,
    input R_add,
    input I_add,
    input I_jalr,
    input l_lbu,
    input l_lw,
    input I_csrrw,
    input I_csrrs,
    input [31:0] rdata_1,
    input [31:0] rdata_2,
    input [31:0] imm,
    input [31:0] pc,
    output reg [31:0] csr_wdata,
    output reg [31:0] ALU_OUT
);
    reg [31:0] A;
    reg [31:0] B;

    always @(*) begin 
        A = 0;
        B = 0;
        ALU_OUT = 0;
        if(I_csrrw |I_csrrs) begin
            csr_wdata = rdata_1;
        end
        else begin 
            csr_wdata = 32'b0;
        end

        if (R_TYPE | I_TYPE | S_TYPE) begin
            A = rdata_1;
        end
        else if (B_TYPE | J_TYPE) begin 
            A = pc;
        end
        else if (U_TYPE) begin 
            A = 32'b0;
        end

        if (R_TYPE) begin 
            B = rdata_2;
        end 
        else if (S_TYPE | I_TYPE | B_TYPE | U_TYPE) begin
            B = imm;
        end

        if (R_add | I_add | I_jalr | U_TYPE | S_TYPE | l_lbu|l_lw) begin 
            ALU_OUT = A + B;
        end else begin
            ALU_OUT = 0;
        end
        // $display("A = 0x%08x\n",A);//debug
        // $display("B = 0x%08x\n",B);//debug
        //if(I_jalr) begin     
          //  $display("ALU_OUT = 0x%08x\n",ALU_OUT);//debug
        // $display("I_TYPE = %d\n",I_TYPE);//debug
        //end 
    end
    
endmodule
module ysyx_25080202_IDU(
  input [31:0] inst,
  output reg R_TYPE,
  output reg I_TYPE_ARITH,
  output reg L_TYPE_LOAD,
  output reg S_TYPE,
  output reg U_TYPE,
  output reg I_TYPE,
  output reg MemWEn,
  output reg B_TYPE,
  output reg J_TYPE,
  output reg I_jalr,
  output reg U_lui,
  output reg R_add,
  output reg l_lw,
  output reg l_lbu,
  output reg I_add,
  output reg S_sw,
  output reg S_sb,
  output reg I_ebreak,
  output reg I_csrrw,
  output reg I_csrrs,
  output reg [11:0]csr_addr,
  output reg [31:0] imm,
  output reg [4:0]r1,
  output reg [4:0]r2,
  output reg [4:0]rd,
  output reg [3:0]wmask
); 
  wire [6:0] opcode = inst[6:0];
  wire [2:0] funct3 = inst[14:12];
  wire [6:0] funct7 = inst[31:25];

  wire [31:0] immI = {{20{inst[31]}},inst[31:20]};
  wire [31:0] immS = {{20{inst[31]}},inst[31:25],inst[11:7]};
  wire [31:0] immB = {{20{inst[31]}},inst[7],inst[30:25],inst[11:8],1'b0};
  wire [31:0] immU = {inst[31:12],12'b000000000000};
  wire [31:0] immJ = {{12{inst[31]}},inst[19:12],inst[20],inst[30:21],1'b0};
  always @(*) begin
    R_TYPE = 0;
    I_TYPE_ARITH = 0;
    L_TYPE_LOAD = 0;
    S_TYPE = 0;
    MemWEn = 0;
    B_TYPE = 0;
    J_TYPE = 0;
    I_jalr = 0;
    U_lui = 0;
    R_add = 0;
    l_lw = 0;
    l_lbu = 0;
    I_add = 0;
    S_sw = 0;
    S_sb = 0;
    imm = 0;
    I_TYPE = 0;  
    U_TYPE = 0;
    I_csrrw = 0;
    I_csrrs = 0;
    wmask = 4'b0;
    csr_addr = inst[31:20];
    // $display("CSR read addr = %h", csr_addr);
    r1 = inst[19:15];
    r2 = inst[24:20];
    rd = inst[11:7];
    // $display("inst: %b", inst);
    // $display("opcode: %b", opcode);
    case (opcode)
        //R_type指令 (add)
        7'b0110011:begin 
            R_TYPE = 1;
            if(funct3 == 3'b000 && funct7 == 7'b0000000) begin 
                R_add = 1;
            end 
        end
        // I-type算术指令 (addi)//&& funct7 == 7'b0000000
        7'b0010011:begin
            I_TYPE_ARITH = 1;
            if(funct3 == 3'b000) begin 
                I_add = 1;
            end
        end
        7'b0000011:begin
            L_TYPE_LOAD = 1;
            if(funct3 == 3'b010) begin
                l_lw = 1;//LW
            end else if(funct3 == 3'b100) begin 
                l_lbu =1;//LBU
            end
        end 
        7'b0100011:begin
            S_TYPE = 1;
            MemWEn = 1;
            if(funct3 == 3'b010) begin
                S_sw = 1;//SW
                wmask = 4'b1111;
            end
            if(funct3 == 3'b000) begin 
                S_sb = 1;//SB
                wmask =4'b0001;
            end
        end
        7'b1100011:begin
            B_TYPE = 1;
        end 
        7'b1101111:begin
            J_TYPE = 1;
        end
        7'b1100111:begin
            I_jalr =1;//JALR
        end
        7'b0110111:begin
            U_lui = 1;//LUI
        end
        7'b1110011:begin
            if(funct3 == 3'b001) begin 
                I_csrrw = 1;
            end else if(funct3 == 3'b010) begin
                I_csrrs = 1;
            end else if(inst[31:7] == 25'b0000000000010000000000000) begin
                I_ebreak = 1;
            end
        end
        default :begin
             //$display("Unknown opcode: %d", opcode);
        end 
    endcase
    //$display("addi_TYPE: %b", I_add);//调试
    I_TYPE = I_TYPE_ARITH | L_TYPE_LOAD | I_jalr;
    U_TYPE = U_lui;
    // if(I_csrrw == 1) begin
    //     $display("IDU: inst=%h opcode=%b I_csrrw=%b csr_addr=%h", inst, opcode, I_csrrw, csr_addr);
    // end
    case(1'b1)
      I_TYPE: imm = immI;
      S_TYPE: imm = immS;
      B_TYPE: imm = immB;
      U_TYPE: imm = immU;
      J_TYPE: imm = immJ;
      default: begin
            imm = 0;
      end
    endcase

  end

endmodule
module ysyx_25080202_IFU(
    input  clk,
    input  rst,
    input  [31:0] PC,
    input pc_valid,
    input lsu_ready,
    input wbu_ready,
    output reg [31:0] inst,
    output reg ifu_valid,
    output reg ifu_wen,
    output reg        ifu_reqValid,
    input             ifu_respValid,
    output reg [31:0] ifu_raddr,
    input  [31:0]     ifu_rdata
);
    localparam IFU_IDLE = 1'b0;
    localparam IFU_WAIT = 1'b1;
    reg ifu_state;

    always @(posedge clk) begin
        if (rst) begin
            ifu_state    <= IFU_IDLE;
            inst         <= 32'h0; // NOP
            ifu_valid <= 1'b0;
            ifu_raddr <= 32'h30000000;
            ifu_reqValid <= 1'b0;
            //ifu_wen <=1'b0;
        end else begin
            case (ifu_state)
                IFU_IDLE: begin
                    if(pc_valid) begin 
                        ifu_state <= IFU_WAIT; // 发请求后进入 WAIT
                        ifu_raddr <= PC;
                        ifu_reqValid <=1'b1;
                    end 
                    else begin 
                        ifu_state <=IFU_IDLE;
                        //if(wbu_ready || lsu_ready && ifu_valid) begin
                        if((wbu_ready || lsu_ready)&& ifu_valid) begin
                            ifu_valid <= 1'b0;
                        end
                    end
                end
                IFU_WAIT: begin
                    if (ifu_reqValid) begin
                        ifu_reqValid <= 1'b0;
                    end                    
                    if (ifu_respValid) begin
                        ifu_state <= IFU_IDLE;
                        ifu_valid <= 1'b1;
                        inst <= ifu_rdata;
                    end
                end
                default :begin
                    ifu_state <= IFU_IDLE;
                    end
            endcase
        end
    end
endmodule

module ysyx_25080202_LSU(
    input             rst,
    input             clk,
    input      [31:0] R2_data,
    input      [31:0] ALU_OUT,
    //input      [31:0] PC_plus_4,
    input             l_lw,
    input             l_lbu,
    input             S_sb,
    input             S_sw,
    input             R_TYPE,
    input             I_TYPE_ARITH,
    input             I_TYPE,
    input             U_TYPE,
    input             J_TYPE,
    input             I_csrrw,
    input      [31:0] CSR_RDATA,
    input      [4:0]  rd,
    input [3:0] wmask,//IDU传进来的
    // SimpleBus 接口
    input ifu_valid,
    input wbu_ready,
    output reg lsu_valid,
    output reg lsu_reqValid,
    input  lsu_respValid,
    output reg [31:0] lsu_addr,
    output reg lsu_wen,
    output reg [31:0] lsu_wdata,
    output reg [3:0]  lsu_wmask,
    input  [31:0] lsu_rdata,
    output reg lsu_ready,
    output lsu_working,
    output reg [1:0] io_lsu_size,
    // 写回寄存器文件
    output reg [31:0] RegWriteData
);

    // 状态机
    localparam LSU_IDLE = 1'b0;
    localparam LSU_WAIT = 1'b1;
    reg state;

    // 写数据与掩码（组合逻辑）
    wire [31:0] deviation_rdata = lsu_rdata >> (lsu_addr[1:0] * 8);
    // reg [31:0] MemWriteData;
    //reg [3:0]  wmask;


    //assign lsu_addr  = ALU_OUT;
    wire wen  = S_sb | S_sw;
    wire ren  = l_lbu|l_lw; 
    assign lsu_working = ren | wen;

    // 状态迁移（时序逻辑）
    always @(posedge clk) begin
        if (rst) begin
            state        <= LSU_IDLE;
            lsu_ready    <= 1'b0;
            lsu_valid    <= 1'b0;
            lsu_reqValid <= 1'b0;
            lsu_wen      <= 1'b0;
            RegWriteData <=32'b0;
            io_lsu_size <=2'b0;
        end else begin
            case (state)
                LSU_IDLE: begin
                        if (ifu_valid && (wen || ren)) begin
                          //$display("[LSU][%0t] REQ: addr=0x%08h wen=%b ren=%b wdata=0x%08h wmask=0x%1h ALU_OUT=0x%08h", 
         //$time, ALU_OUT, wen, ren, R2_data, wmask, ALU_OUT);
                          state <= LSU_WAIT;
                          lsu_ready<=1'b1;
                          lsu_wen<=wen;
                          //lsu_wdata<= R2_data;
                          lsu_addr <= ALU_OUT;
                          lsu_wmask <= (wmask << ALU_OUT[1:0]);
                          lsu_wdata <= (R2_data << (8 * ALU_OUT[1:0]));
                          //lsu_wdata<= R2_data;
                          lsu_reqValid <= 1'b1;
                          if(S_sb || l_lbu) begin
                            io_lsu_size <= 2'b00;
                          end
                          else if(S_sw || l_lw) begin 
                            io_lsu_size <=2'b10;
                          end else begin 
                            io_lsu_size <= 2'b00;
                          end
                        end
                        else begin
                          state<=LSU_IDLE;
                          if(lsu_valid && wbu_ready) begin
                            lsu_valid <= 1'b0;
                          end
                        end
                      end
                LSU_WAIT: begin
                    if(lsu_ready) begin
                        lsu_ready <= 1'b0;
                    end
                
                    if(lsu_reqValid) begin
                        lsu_reqValid <= 1'b0;
                    end

                    if (lsu_respValid) begin
                        lsu_valid <= 1'b1;
                        state <= LSU_IDLE;
                        if(lsu_wen) begin
                            lsu_wen <= 1'b0;
                        end
                        if(l_lw) begin
                            RegWriteData <= deviation_rdata;
                        end else if(l_lbu) begin
                            // case(ALU_OUT[1:0])
                            //     2'b00: RegWriteData = {24'b0, lsu_rdata[7:0]};
                            //     2'b01: RegWriteData = {24'b0, lsu_rdata[15:8]};
                            //     2'b10: RegWriteData = {24'b0, lsu_rdata[23:16]};
                            //     2'b11: RegWriteData = {24'b0, lsu_rdata[31:24]};
                            // endcase
                            RegWriteData <= {24'b0, deviation_rdata[7:0]};
                            end else begin 
                                RegWriteData <=0;
                        end
                end    
            end                
            endcase
        end
    end

endmodule

module ysyx_25080202_PC (
    input clk,
    input rst,
    input [31:0] next_pc,
    input wbu_valid,
    output reg pc_valid,
    output reg [31:0] pc
);
    always @(posedge clk) begin
        if (rst) begin
            pc <= 32'h30000000;  // 起始地址
            pc_valid <= 1;
        end
        else if (wbu_valid) begin
            pc <= next_pc;
            pc_valid <= 1;
        end
        else if(pc_valid) begin
            pc_valid <= 0;
        end
    end
endmodule
module ysyx_25080202_RegisterFile #(ADDR_WIDTH = 5, DATA_WIDTH = 32) (
  input clk,
  input [DATA_WIDTH-1:0] wdata,
  input [ADDR_WIDTH-1:0] waddr,
  input L_wen,
  input [ADDR_WIDTH-1:0] raddr_1,
  input [ADDR_WIDTH-1:0] raddr_2,
  output reg [DATA_WIDTH-1:0] rdata_1,
  output reg [DATA_WIDTH-1:0] rdata_2,
  
  // 新增：调试寄存器输出端口
  output reg [DATA_WIDTH-1:0] zero,
  output reg [DATA_WIDTH-1:0] ra,
  output reg [DATA_WIDTH-1:0] sp,
  output reg [DATA_WIDTH-1:0] gp,
  output reg [DATA_WIDTH-1:0] tp,
  output reg [DATA_WIDTH-1:0] s0,
  output reg [DATA_WIDTH-1:0] s1,
  output reg [DATA_WIDTH-1:0] a0,
  output reg [DATA_WIDTH-1:0] a1,
  output reg [DATA_WIDTH-1:0] a2,
  output reg [DATA_WIDTH-1:0] a3,
  output reg [DATA_WIDTH-1:0] a4,
  output reg [DATA_WIDTH-1:0] a5  
);
  
    // 寄存器定义（RISC-V ABI名称）
    localparam ZERO = 0;
    localparam RA   = 1;
    localparam SP   = 2;
    localparam GP   = 3;
    localparam TP   = 4;
    localparam S0   = 8;
    localparam S1   = 9;
    localparam A0   = 10;
    localparam A1   = 11;
    localparam A2   = 12;
    localparam A3   = 13;
    localparam A4   = 14;
    localparam A5   = 15;
//     reg wen;
//   assign wen = L_wen;
reg wen;
always @(*) begin
  wen = L_wen;
end
  reg [DATA_WIDTH-1:0] rf [2**(ADDR_WIDTH-1)-1:0];

  always @(posedge clk) begin
    if (wen) rf[waddr[3:0]] <= wdata;
    //f (waddr == 5'd2) $display("[REG] x2(sp) <= 0x%08h", wdata);

  end

    assign rdata_1 = (raddr_1 == 0) ? 32'b0 : rf[raddr_1[3:0]];
    assign rdata_2 = (raddr_2 == 0) ? 32'b0 : rf[raddr_2[3:0]];
    
    // 调试寄存器输出    
    assign zero = (ZERO == 0) ? 32'b0 : rf[ZERO]; // x0 始终为0
    assign ra   = rf[RA];
    assign sp   = rf[SP];
    assign gp   = rf[GP];
    assign tp   = rf[TP];
    assign s0   = rf[S0];
    assign s1   = rf[S1];
    assign a0   = rf[A0];
    assign a1   = rf[A1];
    assign a2   = rf[A2];
    assign a3   = rf[A3];
    assign a4   = rf[A4];
    assign a5   = rf[A5];
// `ifdef VERILATOR
// import "DPI-C" function void set_gpr_ptr(input logic [31:0] a[]);
//   initial begin 
//     set_gpr_ptr(rf);
//   end
// `endif
endmodule
module ysyx_25080202_WBU(
    input clk,
    input rst,
    input [31:0] ALU_OUT,
    input [31:0] CSR_RDATA,
    input [31:0] i_csr_wdata,
    input l_lw,
    input l_lbu,
    input I_jalr,
    input S_sb,
    input S_sw,
    input I_add,
    input R_add,
    input U_lui,
    input I_csrrw,
    input I_csrrs,
    input [31:0] load_wdata,
    input lsu_busy,
    input lsu_valid,
    input ifu_valid,
    input [31:0] PC,
    output reg wbu_ready,
    output reg wbu_valid,
    output [31:0] next_pc,
    output [31:0] reg_wdata,
    output [31:0] csr_wdata
);

    localparam IDLE = 2'b00;
    localparam WAIT = 2'b01;
    localparam LSUWAIT = 2'b10; 
    reg [1:0] state;
    assign next_pc = (I_jalr) ? {ALU_OUT[31:1],1'b0} : PC+4; 
    assign reg_wdata = (I_jalr) ? PC + 4 : 
                        (I_csrrw | I_csrrs) ? CSR_RDATA:
                        (lsu_valid) ? load_wdata : ALU_OUT;
    assign csr_wdata = i_csr_wdata;          

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            //next_pc <= 32'h30000004;            
            wbu_ready <= 1'b0;
            wbu_valid <= 1'b0;
            //reg_wdata <= 32'b0;
            //csr_wdata <= 32'b0;
        end else begin
            case (state)
                IDLE: begin
                    if (ifu_valid) begin
                        if (lsu_busy) begin
                            state <= LSUWAIT;
                            wbu_valid <= 1'b0;
                        end
                        else begin
                            state <= WAIT;
                            wbu_ready <= 1'b1;
                            
                                wbu_valid <=1'b1;
                            
                        end
                    end
                    else begin
                        state <= IDLE;
                        if (wbu_ready) begin
                            wbu_ready <= 1'b0;
                        end
                        if (wbu_valid) begin
                            wbu_valid <= 1'b0;
                        end
                    end
                end                           
                WAIT: begin
                    if (wbu_ready) begin
                        wbu_ready <= 1'b0;
                    end
                    if(I_jalr) begin
                        //next_pc <= {ALU_OUT[31:1],1'b0};
                        //reg_wdata <= PC + 4;
                    end else if(I_csrrw) begin
                        //csr_wdata <= ALU_OUT;
                        //reg_wdata <=CSR_RDATA;
                        //next_pc <=PC + 4;
                    end
                    else begin
                        //next_pc <=PC + 4;
                        //reg_wdata <= ALU_OUT;
                    end
                    state <= IDLE;
                    wbu_valid <= 1'b0;

                end
                LSUWAIT: begin
                    if (lsu_valid) begin
                        wbu_ready <= 1'b1;
                        //reg_wdata <= load_wdata;
                        state <= IDLE;
                        //next_pc <= PC + 4;  
                              
                        wbu_valid <= 1'b1;                
                    end
                end
                default:begin
                    state<=IDLE;
                    end
            endcase
        end
    end

endmodule
module ysyx_25080202(
    input clock,
    input reset,
    input io_ifu_respValid,
    input [31:0]io_ifu_rdata,
    output [31:0] io_ifu_addr,
    output io_ifu_reqValid,
    input [31:0]  io_lsu_rdata,
    input  io_lsu_respValid,
    output io_lsu_reqValid,
    output [31:0] io_lsu_addr,
    output io_lsu_wen,
    output [31:0] io_lsu_wdata,
    output [3:0]  io_lsu_wmask,
    output [1:0]  io_lsu_size

);
    // ===============================
    // 内部信号定义
    // ===============================
    wire        I_csrrw;
    wire        I_csrrs;
    wire [11:0] csr_addr;
    wire [31:0] csr_wdata;
    wire [31:0] csr_rdata;
    wire [31:0] PC_plus_4;
    wire [31:0] ALU_OUT;
    wire [31:0] R1_data;
    wire [31:0] R2_data;
    wire [31:0] RegWriteData;
    wire Reg_WE;
    wire I_jalr;
    wire [4:0] r1, r2, rd;
    wire [31:0] imm;
    wire R_TYPE, I_TYPE_ARITH, L_TYPE_LOAD, S_TYPE, U_TYPE, I_TYPE;
    wire B_TYPE, J_TYPE, U_lui, R_add, l_lw, l_lbu, I_add, S_sw, S_sb, I_ebreak;
    wire [3:0] wmask;
    // ===== IFU ↔ Memory =====
    //wire [31:0] ifu_raddr;
    //reg [31:0] ifu_rdata;
    reg        ifu_reqValid;
    reg        ifu_respValid;
    wire        ifu_valid;
    wire        pc_valid;
    wire [31:0] PC;
    wire        cpu_en;
    // ===== LSU ↔ Memory =====
   // wire [31:0] lsu_addr;
    //wire [31:0] lsu_wdata;
   // wire [3:0]  lsu_wmask;
   // wire [31:0] lsu_rdata;
    // wire        lsu_wen;
    // wire        lsu_reqValid;
    // wire        lsu_respValid;
    wire        lsu_ready;
    wire        lsu_working;
    wire        lsu_valid;
    // ===== WBU 控制信号 =====
    wire        wbu_ready;
    wire        wbu_valid;
    wire [31:0] next_pc;
    wire [31:0] reg_wdata;
    wire [31:0] csr_wdata_out;
    wire [31:0] inst;

   // wire ifu_wen;

    // ===============================
    // 指令存储器仿真接口
    // ===============================
    //import "DPI-C" function int pmem_read(input int raddr);

    ysyx_25080202_IFU ifu (
        .clk(clock),
        .rst(reset),
        .PC(PC),
        .inst(inst),
        .pc_valid(pc_valid),
        .wbu_ready(wbu_ready),
        .ifu_valid(ifu_valid),
        .ifu_wen(),
        .ifu_reqValid(io_ifu_reqValid),   // *** 修改：连接握手信号
        .ifu_respValid(io_ifu_respValid),// *** 修改：连接握手信号
        .ifu_raddr(io_ifu_addr),
        .ifu_rdata(io_ifu_rdata),
        .lsu_ready(lsu_ready)
    );


    wire [31:0] load_wdata;
    // `ifdef VERILATOR
    // import "DPI-C" function void notify_ebreak();
    // always @(posedge clock) begin
    //     if (I_ebreak && $time > 0) begin
    //         $display("[TRAP] EBREAK at PC = 0x%08x", PC);
    //         notify_ebreak();
    //     end
    // end
    // `endif

    ysyx_25080202_PC pc(
        .clk(clock),
        .rst(reset),
        .next_pc(next_pc),
        .wbu_valid(wbu_valid),
        .pc_valid(pc_valid),
        .pc(PC)
    );

    ysyx_25080202_IDU idu(
        .inst(inst),
        .R_TYPE(R_TYPE),
        .I_TYPE_ARITH(I_TYPE_ARITH),
        .L_TYPE_LOAD(L_TYPE_LOAD),
        .S_TYPE(S_TYPE),
        .U_TYPE(U_TYPE),
        .I_TYPE(I_TYPE),
        .MemWEn(),
        .B_TYPE(B_TYPE),
        .J_TYPE(J_TYPE),
        .I_jalr(I_jalr),
        .U_lui(U_lui),
        .R_add(R_add),
        .l_lw(l_lw),
        .l_lbu(l_lbu),
        .I_add(I_add),
        .S_sw(S_sw),
        .S_sb(S_sb),
        .I_ebreak(I_ebreak),
        .I_csrrw(I_csrrw),
        .I_csrrs(I_csrrs),
        .csr_addr(csr_addr),
        .imm(imm),
        .r1(r1),
        .r2(r2),
        .rd(rd),
        .wmask(wmask)
    );

    ysyx_25080202_LSU lsu (
        .rst(reset),
        .clk(clock),
        .rd(rd),
        .R2_data(R2_data),
        .ALU_OUT(ALU_OUT),
        .l_lw(l_lw),
        .l_lbu(l_lbu),
        .S_sb(S_sb),
        .S_sw(S_sw),
        .R_TYPE(R_TYPE),
        .I_TYPE_ARITH(I_TYPE_ARITH),
        .U_TYPE(U_TYPE),
        .J_TYPE(J_TYPE),
        .I_csrrw(I_csrrw),
        .CSR_RDATA(csr_rdata),
        //.lsu_en(cpu_en),
        .I_TYPE(I_TYPE),
        .wmask(wmask),
        // ===== SimpleBus 信号 =====
        .ifu_valid(ifu_valid),
        .wbu_ready(wbu_ready),
        .lsu_valid(lsu_valid),
        .lsu_reqValid(io_lsu_reqValid),   // *** 修改：增加 reqValid 信号连接
        .lsu_respValid(io_lsu_respValid), // *** 修改：增加 respValid 信号连接
        .lsu_rdata(io_lsu_rdata),
        .lsu_addr(io_lsu_addr),
        .lsu_wen(io_lsu_wen),
        .lsu_wdata(io_lsu_wdata),
        .lsu_wmask(io_lsu_wmask),
        .lsu_ready(lsu_ready),
        .lsu_working(lsu_working),
        .io_lsu_size(io_lsu_size),
        //.wb_addr(lsu_wb_addr),
        // ===== 写回寄存器 =====
        .RegWriteData(load_wdata)
        //.Reg_WE(Reg_WE)
    );

    ysyx_25080202_RegisterFile regfile (
        .clk(clock),
        .wdata(reg_wdata),
        .waddr(rd),
        .L_wen(wbu_valid && !(S_sb||S_sw)),
        .raddr_1(r1),
        .raddr_2(r2),
        .rdata_1(R1_data),
        .rdata_2(R2_data),
        .zero(),
        .ra(),
        .sp(),
        .gp(),
        .tp(),
        .s0(),
        .s1(),
        .a0(),
        .a1(),
        .a2(),
        .a3(),
        .a4(),
        .a5()
    );

    ysyx_25080202_EXU alu (
        .R_TYPE(R_TYPE),
        .I_TYPE(I_TYPE),
        .S_TYPE(S_TYPE),
        .B_TYPE(B_TYPE),
        .J_TYPE(J_TYPE),
        .U_TYPE(U_TYPE),
        .R_add(R_add),
        .I_add(I_add),
        .I_jalr(I_jalr),
        .l_lbu(l_lbu),
        .l_lw(l_lw),
        .I_csrrw(I_csrrw),
        .I_csrrs(I_csrrs),
        .rdata_1(R1_data),
        .rdata_2(R2_data),
        .imm(imm),
        .pc(PC),
        .csr_wdata(csr_wdata),
        .ALU_OUT(ALU_OUT)
    );

    ysyx_25080202_CSR csr(
        .clk(clock),
        .rst(reset),
        .I_csrrw(I_csrrw),
        .I_csrrs(I_csrrs),
        .csr_addr(csr_addr),
        .csr_wdata(csr_wdata_out),
        .csr_rdata(csr_rdata)
    );
    ysyx_25080202_WBU wbu (
        .clk(clock),
        .rst(reset),
        .ALU_OUT(ALU_OUT),
        .CSR_RDATA(csr_rdata),
        .i_csr_wdata(csr_wdata),
        .l_lw(l_lw),
        .l_lbu(l_lbu),
        .I_jalr(I_jalr),
        .S_sb(S_sb),
        .S_sw(S_sw),
        .I_add(I_add),
        .R_add(R_add),
        .U_lui(U_lui),
        .I_csrrw(I_csrrw),
        .I_csrrs(I_csrrs),
        .load_wdata(load_wdata),
        .lsu_busy(lsu_working),
        .lsu_valid(lsu_valid),
        .ifu_valid(ifu_valid),
        .PC(PC),
        .wbu_ready(wbu_ready),
        .wbu_valid(wbu_valid),
        .next_pc(next_pc),
        .reg_wdata(reg_wdata),
        .csr_wdata(csr_wdata_out)
    );
endmodule

