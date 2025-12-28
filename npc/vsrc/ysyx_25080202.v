/******************ifu********************/
module ysyx_25070198_ifu(
    input clock,
    input reset,

    input [31:0] jump_pc,
    input jump,
    input io_lsu_respValid,
    input mem_ren,
    input inst_done,
    
    output reg [31:0] pc,
    output [31:0] inst,
    output reg inst_valid,
    
    output reg [31:0] ifu_raddr,
    input [31:0] io_ifu_rdata,

    output reg io_ifu_reqValid,
    input io_ifu_respValid
);

    reg ifu_reqValid;
    assign io_ifu_reqValid = ifu_reqValid & !reset;

    //状态定义
    typedef enum logic [1:0] {
        IFU_IDLE = 2'b00,
        IFU_WAIT = 2'b01,
        IFU_STAY = 2'b10  //wait for io_ifu_respValid
    } ifu_state_t;
    
    ifu_state_t ifu_current_state, ifu_next_state;
    
    always @(posedge clock) begin
        if (reset) begin
            pc <= 32'h30000000;
        end 
        else if (jump) begin
            pc <= jump_pc;
        end
        else if (inst_done) begin
            pc <= pc + 32'h4; 
        end
    end
    
    //状态寄存器
    always @(posedge clock) begin
        if (reset) begin
            ifu_current_state <= IFU_IDLE;
        end else begin
            ifu_current_state <= ifu_next_state;
        end
    end
    
    reg [31:0] inst_reg;
    reg [31:0] inst_next;
    always @(posedge clock) begin
        if (reset) begin
            inst_reg <= 32'h0;
        end else begin
            inst_reg <= inst_next;
        end
    end
    assign inst = inst_reg;

    //输出逻辑和下一状态逻辑
    always @(*) begin
        case (ifu_current_state)
            IFU_IDLE: begin   //00   发送取指请求
                ifu_reqValid = 1'b1;
                ifu_raddr = pc;
                inst_valid = 1'b0;
                inst_next = 32'h0;
                ifu_next_state = IFU_STAY;
            end

            IFU_STAY: begin   //10   等待响应
                ifu_reqValid = 1'b0;
                ifu_raddr = pc;
                inst_valid = 1'b0;
                
                //收到响应，进入WAIT状态，准备执行指令
                if (io_ifu_respValid) begin
                    ifu_next_state = IFU_WAIT;
                    inst_next = io_ifu_rdata;
                end else begin
                    ifu_next_state = IFU_STAY;
                    inst_next = inst_reg;
                end
            end

            IFU_WAIT: begin   //01   等待执行完成
                ifu_reqValid = 1'b0;
                ifu_raddr = pc;
                inst_valid = 1'b1;
                inst_next = inst_reg;

                //如果指令执行完成，进入IDLE状态，准备取下一条指令
                if (inst_done) begin 
                    ifu_next_state = IFU_IDLE;
                end else begin
                    ifu_next_state = IFU_WAIT;
                end
            end
            
            default: begin
                ifu_reqValid = 1'b0;
                ifu_raddr = 32'h0;
                inst_valid = 1'b0;
                inst_next = 32'h0;
                ifu_next_state = IFU_IDLE;
            end
        endcase
    end

endmodule

/******************idu********************/
module ysyx_25070198_idu(
    input clock,
    input reset,

    input [31:0] inst,

    output [4:0] rs1,
    output [4:0] rs2,
    output [4:0] rd,
    output [31:0] imm,

    output [1:0] io_lsu_size,

    output is_addi,
    output is_jalr,
    output is_add,
    output is_lui,
    output is_lw,
    output is_lbu,
    output is_sw,
    output is_sb,
    
    output is_csrrw

);

    wire [6:0] opcode;
    wire [2:0] funct3;
    wire [6:0] funct7;

    assign opcode = inst[6:0];
    assign rd = inst[11:7];
    assign funct3 = inst[14:12];
    assign rs1 = inst[19:15];
    assign rs2 = inst[24:20];
    assign funct7 = inst[31:25];

    assign is_addi = (opcode == 7'b0010011) && (funct3 == 3'b000);    //I
    assign is_jalr = (opcode == 7'b1100111) && (funct3 == 3'b000);    //I
    assign is_add  = (opcode == 7'b0110011) && (funct3 == 3'b000);    //R
    assign is_lui  = (opcode == 7'b0110111);                          //U
    assign is_lw   = (opcode == 7'b0000011) && (funct3 == 3'b010);    //I
    assign is_lbu  = (opcode == 7'b0000011) && (funct3 == 3'b100);    //I
    assign is_sw   = (opcode == 7'b0100011) && (funct3 == 3'b010);    //S
    assign is_sb   = (opcode == 7'b0100011) && (funct3 == 3'b000);    //S

    assign is_csrrw = (opcode == 7'b1110011) && (funct3 == 3'b001);   //I

    assign io_lsu_size = (is_lw || is_sw) ? 2'b10 : 
                         (is_lbu || is_sb) ? 2'b00 :
                         2'b00;
    
    wire [31:0] i_imm = {{20{inst[31]}}, inst[31:20]};
    wire [31:0] s_imm = {{20{inst[31]}}, inst[31:25], inst[11:7]};
    wire [31:0] u_imm = {inst[31:12], 12'b0};
    //r型无立即数 有funct7
    wire [31:0] csr_imm = {20'b0, inst[31:20]};

    assign imm = (is_addi || is_jalr || is_lw || is_lbu ) ? i_imm :
                 (is_lui) ? u_imm :
                 (is_sw || is_sb) ? s_imm :
                 (is_csrrw) ? csr_imm :
                 32'b0;
   
endmodule


/******************exu********************/
module ysyx_25070198_exu(
    input clock,
    input reset,

    input inst_valid,

    input is_addi,
    input is_jalr,
    input is_add,
    input is_lui,
    input is_lw,
    input is_lbu,
    input is_sw,
    input is_sb,

    input is_csrrw,
    input [31:0] csr_rdata,
    output csr_wen,
    output [11:0] csr_addr,

    input io_lsu_respValid,

    input [31:0] pc,
    input [31:0] reg_rdata1,reg_rdata2,imm,
    output mem_ren,mem_wen,reg_wen,reg_men,
    output [31:0] reg_wdata,mem_wdata,
    output [31:0] mem_addr,
    output [3:0] mem_mask, 
    output [1:0] sel,

    output [31:0] jump_pc,
    output jump
);
assign jump_pc = is_jalr ? (reg_rdata1 + imm) & 32'hFFFFFFFE : 32'b0;
assign jump = is_jalr && inst_valid;

assign reg_wen = ((is_add || is_addi || is_jalr || is_lui || is_csrrw) && inst_valid) || 
                 ((is_lw || is_lbu) && io_lsu_respValid);
assign reg_men = (is_lw || is_lbu) && inst_valid;

assign mem_ren = (is_lw || is_lbu) && inst_valid && !io_lsu_respValid;
assign mem_wen = (is_sw || is_sb) && inst_valid;

assign sel = {reg_rdata1 + imm}[1:0];

assign mem_addr = (mem_ren || mem_wen) ? {reg_rdata1 + imm}[31:0] : 32'b0;
assign mem_mask = (is_sb) ? (4'b0001 << sel):
                    (is_sw) ? 4'b1111 :
                    4'b0;

assign reg_wdata = (is_jalr) ? pc + 32'h4 : 
                    (is_addi) ? reg_rdata1 + imm :
                    (is_add) ? reg_rdata1 + reg_rdata2 :
                    (is_lui) ? imm :
                    (is_csrrw) ? csr_rdata :
                    32'b0;

assign mem_wdata = (is_sw) ? reg_rdata2 : 
                    (is_sb && mem_mask == 4'd1) ? {24'b0, reg_rdata2[7:0]} :
                    (is_sb && mem_mask == 4'd2) ? {16'b0, reg_rdata2[7:0], 8'b0} :
                    (is_sb && mem_mask == 4'd4) ? {8'b0, reg_rdata2[7:0], 16'b0} :
                    (is_sb && mem_mask == 4'd8) ? {reg_rdata2[7:0], 24'b0} :
                    0;

assign csr_wen = is_csrrw && inst_valid;
assign csr_addr = imm[11:0];

endmodule


/******************reg********************/
module ysyx_25070198_rf(
    input clock,
    input reset,

    input [31:0] reg_wdata,mem_rdata,
    input [4:0] reg_waddr,
    input reg_wen,reg_men,is_lbu,
    input [1:0] sel,

    output reg [31:0] debug_x10,

    input [4:0] reg_raddr1,reg_raddr2,
    output [31:0] reg_rdata1,reg_rdata2

);
reg [31:0] rf [0:15];

wire [7:0] mem_rdatas = (sel == 2'd0) ? mem_rdata[7:0] :
                        (sel == 2'd1) ? mem_rdata[15:8] :
                        (sel == 2'd2) ? mem_rdata[23:16] :
                        (sel == 2'd3) ? mem_rdata[31:24] :
                        0;

integer i;

always@(posedge clock or posedge reset) begin
    if(reset) begin
        for(i=0; i<32; i=i+1) rf[i] <= 0;
    end
    else if(reg_wen && reg_waddr != 5'b0) begin
        // 如果是内存读取操作，使用mem_rdata；否则使用reg_wdata
        if(reg_men) begin
            rf[reg_waddr] <= (is_lbu) ? {24'b0, mem_rdatas} : mem_rdata;
        end else begin
            rf[reg_waddr] <= reg_wdata;
        end
    end
end

assign reg_rdata1 = rf[reg_raddr1];
assign reg_rdata2 = rf[reg_raddr2];

endmodule

/******************lsu********************/
module ysyx_25070198_lsu(
    input clock,
    input reset,
    
    //来自EXU
    input mem_ren,
    input mem_wen,
    input [31:0] mem_addr,
    input [31:0] mem_wdata,
    input [3:0] mem_mask,
    
    //到寄存器堆
    output reg [31:0] mem_rdata,
    
    //SimpleBus接口
    output reg [31:0] io_lsu_addr,
    output reg io_lsu_wen,
    output reg [31:0] io_lsu_wdata,
    output reg [3:0] io_lsu_wmask,
    input [31:0] io_lsu_rdata,

    output reg io_lsu_reqValid,
    input io_lsu_respValid
);

    //状态定义
    typedef enum logic {
        LSU_IDLE = 1'b0,
        LSU_WAIT = 1'b1
    } lsu_state_t;
    
    lsu_state_t lsu_current_state, lsu_next_state;

    //状态寄存器
    always @(posedge clock) begin
        if (reset) begin
            lsu_current_state <= LSU_IDLE;
        end else begin
            lsu_current_state <= lsu_next_state;
        end
    end
    
    //下一状态逻辑
    always @(*) begin
        case (lsu_current_state)
            LSU_IDLE: begin    //0
                if (mem_ren || mem_wen) begin
                    lsu_next_state = LSU_WAIT;  //读写操作->等待
                end else begin
                    lsu_next_state = LSU_IDLE;
                end
            end

            LSU_WAIT: begin    //1
                if(io_lsu_respValid) 
                    lsu_next_state = LSU_IDLE;  //完成返回IDLE
                else
                    lsu_next_state = LSU_WAIT;  //继续等待
            end
            
            default: begin
                lsu_next_state = LSU_IDLE;
            end
        endcase
    end
    
    //输出逻辑
    always @(*) begin
        case (lsu_current_state)
            LSU_IDLE: begin   //0
                io_lsu_reqValid = mem_ren || mem_wen;
                io_lsu_addr = mem_addr;
                io_lsu_wen = mem_wen;
                io_lsu_wdata = mem_wdata;
                io_lsu_wmask = mem_mask;
                mem_rdata = 32'b0;
            end
            
            LSU_WAIT: begin   //1
                io_lsu_reqValid = 1'b1;
                io_lsu_addr = mem_addr;
                io_lsu_wen = mem_wen;
                io_lsu_wdata = mem_wdata;
                io_lsu_wmask = mem_mask;
                mem_rdata = io_lsu_rdata;
            end

            default: begin
                io_lsu_reqValid = 1'b0;
                io_lsu_addr = 32'b0;
                io_lsu_wen = 1'b0;
                io_lsu_wdata = 32'b0;
                io_lsu_wmask = 4'b0;
                mem_rdata = 32'b0;
            end
        endcase
    end

endmodule

/******************csr********************/
module ysyx_25070198_csr_reg (
    input clock,
    input reset,
    input csr_wen,
    input [11:0] csr_addr,
    input [31:0] csr_wdata,//rs1 reg_rdata1
    output [31:0] csr_rdata
);
    reg [31:0] mcycle;
    reg [31:0] mcycleh;
    reg [31:0] mvendorid;
    reg [31:0] marchid;

    always @(posedge clock) begin
        if (reset) begin
            {mcycleh, mcycle} <= 64'b0;
            mvendorid <= 32'h79737978;
            marchid <= 32'd25070198;
        end
        else if (csr_wen) begin
            case (csr_addr)
                12'hB00: mcycle  <= csr_wdata;
                12'hB80: mcycleh <= csr_wdata;
                default: {mcycleh, mcycle} <= {mcycleh, mcycle} + 64'b1;
            endcase
        end
        else begin
            {mcycleh, mcycle} <= {mcycleh, mcycle} + 64'b1;
        end
    end

    assign csr_rdata = (csr_addr == 12'hB00) ? mcycle :
                      (csr_addr == 12'hB80) ? mcycleh :
                      (csr_addr == 12'hF11) ? mvendorid :
                      (csr_addr == 12'hF12) ? marchid :
                      32'b0;


endmodule

/******************top********************/
module ysyx_25080202(
    input clock,
    input reset,

    output io_ifu_reqValid,
    output [31:0] io_ifu_addr,
    input io_ifu_respValid,
    input [31:0] io_ifu_rdata,

    output io_lsu_reqValid,
    output [31:0] io_lsu_addr,

    output [1:0] io_lsu_size,

    output io_lsu_wen,
    output [31:0] io_lsu_wdata,
    output [3:0] io_lsu_wmask,
    input io_lsu_respValid,
    input [31:0] io_lsu_rdata

);

    // IFU接口
    wire [31:0] pc;
    wire [31:0] jump_pc;
    wire jump;
    wire inst_valid;
    
    // IDU接口
    wire [31:0] inst;
    wire [4:0] rs1, rs2, rd;
    wire [31:0] imm;
    wire is_addi, is_jalr, is_add, is_lui, is_lw, is_lbu, is_sw, is_sb, is_csrrw;

    // EXU接口
    wire [31:0] reg_rdata1, reg_rdata2;
    wire mem_ren, mem_wen, reg_wen, reg_men;
    wire [31:0] reg_wdata, mem_wdata;
    wire [31:0] mem_addr;
    wire [3:0] mem_mask;
    wire [1:0] sel;

    // CSR接口
    wire csr_wen;
    wire [11:0] csr_addr;
    wire [31:0] csr_rdata;

    // LSU接口
    wire inst_done;
    
    // 内存接口
    wire [31:0] mem_rdata;
    
    // SimpleBus 接口
    wire [31:0] ifu_raddr;
    assign io_ifu_addr = ifu_raddr;

    // 指令执行完成
    assign inst_done = inst_valid && (!(is_lw || is_lbu || is_sb || is_sw) || io_lsu_respValid);

    ysyx_25070198_ifu ysyx_25070198_ifu0(
        .clock(clock),
        .reset(reset),
        .jump_pc(jump_pc),
        .jump(jump),
        .pc(pc),
        .inst(inst),
        .inst_valid(inst_valid),
        .inst_done(inst_done),
        .io_lsu_respValid(io_lsu_respValid),
        .mem_ren(mem_ren),
        .io_ifu_rdata(io_ifu_rdata),
        .ifu_raddr(ifu_raddr),
        .io_ifu_reqValid(io_ifu_reqValid),
        .io_ifu_respValid(io_ifu_respValid)
    );
    
    ysyx_25070198_idu ysyx_25070198_idu0(
        .clock(clock),
        .reset(reset),
        .inst(inst),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .imm(imm),
        .is_addi(is_addi),
        .is_jalr(is_jalr),
        .is_add(is_add),
        .is_lui(is_lui),
        .is_lw(is_lw),
        .is_lbu(is_lbu),
        .is_sw(is_sw),
        .is_sb(is_sb),
        
        .is_csrrw(is_csrrw),
        .io_lsu_size(io_lsu_size)
    );

    ysyx_25070198_exu ysyx_25070198_exu0(
        .clock(clock),
        .reset(reset),
        .is_addi(is_addi),
        .is_jalr(is_jalr),
        .is_add(is_add),
        .is_lui(is_lui),
        .is_lw(is_lw),
        .is_lbu(is_lbu),
        .is_sw(is_sw),
        .is_sb(is_sb),
        .pc(pc),
        .reg_rdata1(reg_rdata1),
        .reg_rdata2(reg_rdata2),
        .imm(imm),
        .mem_ren(mem_ren),
        .mem_wen(mem_wen),
        .reg_wen(reg_wen),
        .reg_men(reg_men),
        .reg_wdata(reg_wdata),
        .mem_wdata(mem_wdata),
        .mem_addr(mem_addr),
        .mem_mask(mem_mask),
        .sel(sel),
        .jump_pc(jump_pc),
        .jump(jump),

        .inst_valid(inst_valid),
        .io_lsu_respValid(io_lsu_respValid),

        .is_csrrw(is_csrrw),
        .csr_rdata(csr_rdata),
        .csr_wen(csr_wen),
        .csr_addr(csr_addr)
    );

    ysyx_25070198_rf ysyx_25070198_rf0(
        .clock(clock),
        .reset(reset),
        .reg_wdata(reg_wdata),
        .mem_rdata(mem_rdata),
        .reg_waddr(rd),
        .reg_wen(reg_wen),
        .reg_men(reg_men),
        .is_lbu(is_lbu),
        .sel(sel),
        .reg_raddr1(rs1),
        .reg_raddr2(rs2),
        .reg_rdata1(reg_rdata1),
        .reg_rdata2(reg_rdata2),
        .debug_x10(debug_x10)
    );

    ysyx_25070198_csr_reg ysyx_25070198_csr_reg0(
        .clock(clock),
        .reset(reset),
        .csr_wen(csr_wen),
        .csr_addr(csr_addr),
        .csr_wdata(reg_rdata1),
        .csr_rdata(csr_rdata)
    );

    ysyx_25070198_lsu ysyx_25070198_lsu0(
        .clock(clock),
        .reset(reset),
        .mem_ren(mem_ren),
        .mem_wen(mem_wen),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_mask(mem_mask),
        .mem_rdata(mem_rdata),
        .io_lsu_respValid(io_lsu_respValid),
        .io_lsu_addr(io_lsu_addr),
        .io_lsu_wen(io_lsu_wen),
        .io_lsu_wdata(io_lsu_wdata),
        .io_lsu_wmask(io_lsu_wmask),
        .io_lsu_rdata(io_lsu_rdata),
        .io_lsu_reqValid(io_lsu_reqValid)
    );

endmodule