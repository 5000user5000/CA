//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,                                                         //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata,                                                       //
    // finish procedure                                                                        //
        output              o_finish,                                                           //
    // cache                                                                                    //
        input               i_cache_finish,                                                     //
        output              o_proc_finish                                                       //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
    // PC
    reg [BIT_W-1:0] PC = 32'h00010000;
    wire [BIT_W-1:0] next_PC;
    wire [31:0] pc_4, pc_jump, pc_branch, pc_imm, pc_rs1;

    // IMM
    wire [BIT_W-1:0] imm;

    // Regfile
    wire [4:0] rs1, rs2, rd;
    wire [BIT_W-1:0] rs1_data, rs2_data, rd_data;

    // Control Signal
    wire [6:0] opcode;
    wire branch, mem_read, mem_write, reg_write, mem_to_reg, alu_src, jal_signal, jalr_signal, alu_usePC;
    wire [2:0] alu_op;

    // ALU control
    wire [2:0] func3;
    wire [6:0] func7;
    wire [3:0] alu_signal;

    // ALU
    wire [BIT_W-1:0] alu_input1, alu_input2, alu_result;
    wire alu_branch;
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
    assign o_IMEM_cen = !o_finish; // instruction memory enable
    assign o_IMEM_addr = PC; // 更新instruction memory address
    assign o_DMEM_cen = mem_read | mem_write; // data memory enable
    assign o_DMEM_wen = mem_write; // data memory write enable
    assign o_DMEM_addr = alu_result; // 更新data memory address
    assign o_DMEM_wdata = rs2_data; // 更新data memory write data

    assign opcode = i_IMEM_data[6:0];
    assign func3 = i_IMEM_data[14:12];
    assign func7 = i_IMEM_data[31:25];
    assign rs1 = i_IMEM_data[19:15];
    assign rs2 = i_IMEM_data[24:20];
    assign rd = i_IMEM_data[11:7];

    assign alu_input1 = alu_usePC ? PC : rs1_data;
    assign alu_input2 = alu_src ? imm : rs2_data;

    assign pc_4 = PC+32'd4;
    assign pc_imm = PC+imm;

    // branch
    assign pc_branch = (branch & alu_branch) ? pc_imm : pc_4;

    // jump
    assign pc_rs1 = jalr_signal ? rs1_data : PC;  // jalr
    assign pc_jump = pc_rs1 + imm;
    
    assign next_PC = (jal_signal | jalr_signal) ? pc_jump : pc_branch;

    assign rd_data = mem_to_reg ? i_DMEM_rdata : alu_result;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // Control wire connection
    Control control0(
        .opcode     (opcode),
        .branch     (branch),
        .mem_read   (mem_read),
        .mem_write  (mem_write),
        .reg_write  (reg_write),
        .mem_to_reg (mem_to_reg),
        .alu_src    (alu_src),
        .alu_op     (alu_op),
        .jal_signal (jal_signal),
        .jalr_signal(jalr_signal),
        .alu_usePC  (alu_usePC),
        .o_finish   (o_finish)
    );


    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (reg_write),          
        .rs1    (rs1),                
        .rs2    (rs2),                
        .rd     (rd),                 
        .wdata  (rd_data),
        .rdata1 (rs1_data),           
        .rdata2 (rs2_data)
    );



    // alu_control wire connection
    ALU_control alu_control0(
        .alu_op     (alu_op),
        .func3      (func3),
        .func7      (func7),
        .alu_signal (alu_signal)  // signal to alu to tell which alu action to use
    );

    // alu wire connection
    ALU alu0(
        .alu_input1 (alu_input1),
        .alu_input2 (alu_input2),
        .alu_signal (alu_signal),
        .alu_result    (alu_result),
        .alu_branch   (alu_branch)
    );

    // Imm_Gen wire connection
    Imm_Gen imm_gen0(
        .inst   (i_IMEM_data),
        .imm    (imm)
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // Todo: any combinational/sequential circuit

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
        end
        else begin
            if (opcode == 7'b0000000) begin // NOP
                PC <= PC;
            end
            else begin
                if(i_DMEM_stall == 1'b1) begin
                    PC <= PC;
                end
                else begin 
                    PC <= next_PC;
                end
            end
        end
        // // PC
        // $display("PC = %h, next_PC = %h", PC, next_PC);
        // $display("pc_4 = %h, pc_imm = %h, pc_branch = %h, pc_rs1 = %h, pc_jump = %h", pc_4, pc_imm, pc_branch, pc_rs1, pc_jump);
        // // IMM
        // $display("imm = %h", imm);
        // $display("rs1 = %d, rs2 = %d, rd = %d", rs1, rs2, rd);
        // $display("rs1_data = %d, rs2_data = %d, rd_data = %d", rs1_data, rs2_data, rd_data);
        // $display("alu_input1 = %d, alu_input2 = %d, alu_result = %d", alu_input1, alu_input2, alu_result);
    end
    

endmodule


module Control (opcode, o_finish, branch, mem_read, mem_write, reg_write, mem_to_reg, alu_src, jal_signal, jalr_signal, alu_usePC, alu_op);

    input [6:0] opcode;
    output reg  o_finish, branch, mem_read, mem_write, reg_write, mem_to_reg, alu_src, jal_signal, jalr_signal, alu_usePC;
    output reg [2:0] alu_op;


    localparam  AUIPC = 7'b0010111;
    localparam  JAL = 7'b1101111;
    localparam  JALR = 7'b1100111;
    localparam  BRANCH = 7'b1100011;
    localparam  LOAD = 7'b0000011;
    localparam  STORE = 7'b0100011;
    localparam  OPERATION_IMM = 7'b0010011;
    localparam  OPERATION = 7'b0110011;
    localparam  ECALL = 7'b1110011;

    // alu control input
    localparam  ALUOP_ADD = 3'b000; // alu action is add
    localparam  ALUOP_OPETATION = 3'b001; // need check func3 and func7 to know which alu action to use
    localparam  ALUOP_IMM_OPETATION = 3'b010; // need check func3 to know which alu action to use
    localparam  ALUOP_BRANCH = 3'b011;  // need check func3 to know which alu action to use
    localparam  ALUOP_ADD4 = 3'b100; // alu action is add 4
    localparam  ALUOP_ECALL = 3'b101; // THE end of program
    localparam  ALUOP_DONOTHING = 3'b110; // do nothing


    always @(*) begin
        $display("Control opcode = %b", opcode);
        case(opcode)
            AUIPC: begin // reg[rd] = pc + {imm, 12'b0};
                o_finish = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                jal_signal = 0;
                jalr_signal = 0;
                alu_usePC = 1;
                alu_op = ALUOP_ADD;
                $display("AUIPC\n");
            end
            JAL: begin // reg[rd] = pc + 4; pc = pc + {imm, 12'b0};
                o_finish = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm (4)
                jal_signal = 1;
                jalr_signal = 0;
                alu_usePC = 1;
                alu_op = ALUOP_ADD4;
                $display("JAL\n");
            end
            JALR: begin // reg[rd] = pc + 4; pc = reg[rs1] + {imm, 12'b0};
                o_finish = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm (4)
                jalr_signal = 1;
                alu_usePC = 1;
                alu_op = ALUOP_ADD4;
                $display("JALR\n");
            end
            BRANCH: begin 
                o_finish = 0;
                branch = 1;
                mem_read = 0;
                mem_write = 0;
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 0;
                jal_signal = 0;
                jalr_signal = 0;
                alu_usePC = 0;
                alu_op = ALUOP_BRANCH;
                $display("BRANCH\n");
            end
            LOAD: begin // reg[rd] = M[reg[rs1] + imm];
                o_finish = 0;
                branch = 0;
                mem_read = 1; // need to read memory
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 1; // need to use memory data 
                alu_src = 1; // need to use imm
                jal_signal = 0;
                jalr_signal = 0;
                alu_usePC = 0;
                alu_op = ALUOP_ADD;
                $display("LOAD\n");
            end
            STORE: begin // M[reg[rs1] + imm] = reg[rs2];
                o_finish = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 1; // need to write memory
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                jal_signal = 0;
                jalr_signal = 0;
                alu_usePC = 0;
                alu_op = ALUOP_ADD;
                $display("STORE\n");
            end
            OPERATION_IMM: begin
                o_finish = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                jal_signal = 0;
                jalr_signal = 0;
                alu_usePC = 0;
                alu_op = ALUOP_IMM_OPETATION;
                $display("OP_IMM\n");
            end
            OPERATION: begin
                o_finish = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 0;
                jal_signal = 0;
                jalr_signal = 0;
                alu_usePC = 0;
                alu_op = ALUOP_OPETATION; 
                $display("OP\n");
            end
            ECALL: begin
                o_finish = 1; // finish program
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 0;
                jal_signal = 0;
                jalr_signal = 0;
                alu_usePC = 0;
                alu_op = ALUOP_ECALL;
                $display("ECALL\n");
            end
            default: begin
                o_finish = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 0;
                jal_signal = 0;
                jalr_signal = 0;
                alu_usePC = 0;
                alu_op = ALUOP_DONOTHING;
                $display("default do nothing\n");
            end
        endcase
    end
endmodule

module Imm_Gen (inst, imm);
    localparam BIT_W = 32;
    input  [BIT_W-1:0]  inst;
    output reg [BIT_W-1:0]  imm;

    localparam  AUIPC = 7'b0010111;
    localparam  JAL = 7'b1101111;
    localparam  JALR = 7'b1100111;
    localparam  BRANCH = 7'b1100011;
    localparam  LOAD = 7'b0000011;
    localparam  STORE = 7'b0100011;
    localparam  OPERATION_IMM = 7'b0010011;
    localparam  OPERATION = 7'b0110011;
    localparam  ECALL = 7'b1110011;
    
    always @(*) begin
        case(inst[6:0])
            AUIPC: begin 
                imm = {inst[31:12], 12'b0};
                // $display("AUIPC imm = %d\n", imm);
            end
            JAL: begin 
                if(inst[31] == 1'b0) begin
                    imm = {11'b0, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};
                end
                else begin
                    imm = {{11{1'b1}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}; // 2's complement
                end
                // $display("JAL imm = %d\n", imm);
            end
            JALR: begin 
                imm = {20'b0, inst[31:20]};
                // $display("JALR imm = %d\n", imm);
            end
            BRANCH: begin 
                if(inst[31] == 1'b0) begin
                    imm = {{19{1'b0}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
                end
                else begin
                    imm = {{19{1'b1}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}; // 2's complement
                end
                // $display("BRANCH imm = %d\n", imm);
            end
            LOAD: begin
                imm = {20'b0, inst[31:20]};
                // $display("LOAD imm = %d\n", imm);
            end
            STORE: begin 
                imm = {20'b0, inst[31:25], inst[11:7]};
                // $display("STORE imm = %d\n", imm);
            end
            OPERATION_IMM: begin
                if(inst[31] == 1'b0) begin
                    imm = {{20{1'b0}}, inst[31:20]};
                end
                else begin
                    imm = {{20{1'b1}}, inst[31:20]}; // 2's complement
                end
                // $display("OP_IMM imm = %d\n", imm);
            end
            default: begin
                imm = {32'b0};
                // $display("default imm = %d\n", imm);
            end
        endcase
    end
endmodule






module ALU_control (alu_op, func3, func7, alu_signal);
    input  [2:0]        alu_op;
    input  [2:0]        func3;
    input  [6:0]        func7;
    output reg [3:0]    alu_signal; // signal to alu to tell which alu action to use

    // alu control input
    localparam  ALUOP_ADD = 3'b000; // alu action is add
    localparam  ALUOP_OPETATION = 3'b001; // need check func3 and func7 to know which alu action to use
    localparam  ALUOP_IMM_OPETATION = 3'b010; // need check func3 to know which alu action to use
    localparam  ALUOP_BRANCH = 3'b011;  // need check func3 to know which alu action to use
    localparam  ALUOP_ADD4 = 3'b100; // alu action is add 4
    localparam  ALUOP_ECALL = 3'b101; // THE end of program
    localparam  ALUOP_DONOTHING = 3'b110; // do nothing


    // alu signal
    localparam ALU_AND = 4'b0000;
    localparam ALU_OR  = 4'b0001;
    localparam ALU_ADD = 4'b0010;
    localparam ALU_SUB = 4'b0011;
    localparam ALU_SLL = 4'b0100;
    localparam ALU_SLT = 4'b0101;
    localparam ALU_SRA = 4'b0110;
    localparam ALU_BEQ = 4'b0111;
    localparam ALU_BNE = 4'b1000;
    localparam ALU_BLT = 4'b1001;
    localparam ALU_BGE = 4'b1010;
    localparam ALU_ADD4 = 4'b1011;
    localparam ALU_MUL = 4'b1100;
    localparam ALU_DONOTHING = 4'b1111;

    always @(*) begin
       $display("ALU_control alu_op = %b, func3 = %b, func7 = %b", alu_op, func3, func7);
        case(alu_op)
            ALUOP_ADD: begin
                alu_signal = ALU_ADD; // instr: ld、sd、jalr。alu action is add
            end
            ALUOP_OPETATION: begin
                if (func7[5]==1'b1) begin
                    alu_signal = ALU_SUB; // instr: R-type sub。alu action is sub
                end
                else if(func7[0]==1'b1) begin
                    alu_signal = ALU_MUL; // instr: R-type ，alu action is MUL
                end
                else begin
                    case(func3)
                    3'b000: alu_signal = ALU_ADD;
                    3'b110: alu_signal = ALU_OR;
                    3'b111: alu_signal = ALU_AND;
                    default: alu_signal = ALU_DONOTHING;
                endcase
                end
            end
            ALUOP_IMM_OPETATION: begin
                case(func3)
                    3'b000: alu_signal = ALU_ADD;
                    3'b001: alu_signal = ALU_SLL;
                    3'b010: alu_signal = ALU_SLT;
                    3'b101: alu_signal = ALU_SRA;
                    default: alu_signal = ALU_DONOTHING;
                endcase
            end
            ALUOP_BRANCH: begin
                case (func3)
                    3'b000: alu_signal = ALU_BEQ; // beq
                    3'b001: alu_signal = ALU_BNE; // bne
                    3'b100: alu_signal = ALU_BLT; // blt
                    3'b101: alu_signal = ALU_BGE; // bge
                    default: alu_signal = ALU_DONOTHING;
                endcase
            end
            ALUOP_ADD4: begin
                alu_signal = ALU_ADD4;
            end
            default: begin
                alu_signal = ALU_DONOTHING; // JUST a default value
            end
        endcase
        $display("ALU_control alu_signal = %b\n", alu_signal);
    end
endmodule

module ALU (alu_input1, alu_input2, alu_signal, alu_result, alu_branch);
    localparam BIT_W = 32;

    input  [BIT_W-1:0]  alu_input1;
    input  [BIT_W-1:0]  alu_input2;
    input  [3:0]        alu_signal;
    output reg [BIT_W-1:0]  alu_result;
    output reg  alu_branch; // alu branch signal


    // alu signal
    localparam ALU_AND = 4'b0000;
    localparam ALU_OR  = 4'b0001;
    localparam ALU_ADD = 4'b0010;
    localparam ALU_SUB = 4'b0011;
    localparam ALU_SLL = 4'b0100;
    localparam ALU_SLT = 4'b0101;
    localparam ALU_SRA = 4'b0110;
    localparam ALU_BEQ = 4'b0111;
    localparam ALU_BNE = 4'b1000;
    localparam ALU_BLT = 4'b1001;
    localparam ALU_BGE = 4'b1010;
    localparam ALU_ADD4 = 4'b1011;
    localparam ALU_MUL = 4'b1100;
    localparam ALU_DONOTHING = 4'b1111;


    always @(*) begin
        $display("alu_input1 = %d, alu_input2 = %d\n", alu_input1, alu_input2);
        case(alu_signal)
            ALU_AND: begin
                $display("ALU_AND\n");
                alu_result = alu_input1 & alu_input2;
                alu_branch = 1'b0;
            end
            ALU_OR: begin
                $display("ALU_OR\n");
                alu_result = alu_input1 | alu_input2;
                alu_branch = 1'b0;
            end
            ALU_ADD: begin
                $display("ALU_ADD\n");
                alu_result = alu_input1 + alu_input2;
                alu_branch = 1'b0;
            end
            ALU_SUB: begin
                $display("ALU_SUB\n");
                alu_result = alu_input1 - alu_input2;
                alu_branch = 1'b0;
            end
            ALU_SLL: begin
                $display("ALU_SLL\n");
                alu_result = alu_input1 << alu_input2;
                alu_branch = 1'b0;
            end
            ALU_SLT: begin
                $display("ALU_SLT\n");
                alu_result = (alu_input1 < alu_input2) ? 32'h1 : 32'h0;
                alu_branch = 1'b0;
            end
            ALU_SRA: begin
                $display("ALU_SRA\n");
                alu_result = alu_input1 >>> alu_input2;
                alu_branch = 1'b0;
            end
            ALU_BEQ: begin
                $display("ALU_BEQ\n");
                alu_result = 1'b0;
                alu_branch = (alu_input1 == alu_input2) ? 32'h1 : 32'h0; 
            end
            ALU_BNE: begin
                $display("ALU_BNE\n");
                alu_result = 1'b0;
                alu_branch = (alu_input1 != alu_input2) ? 32'h1 : 32'h0; 
            end
            ALU_BLT: begin
                $display("ALU_BLT\n");
                alu_result = 1'b0;
                alu_branch = (alu_input1 < alu_input2) ? 32'h1 : 32'h0; 
            end
            ALU_BGE: begin
                $display("ALU_BGE\n");
                alu_result = 1'b0;
                alu_branch = (alu_input1 >= alu_input2) ? 32'h1 : 32'h0; 
            end
            ALU_ADD4: begin
                $display("ALU_ADD4\n");
                alu_result = alu_input1 + 32'h4;
                alu_branch = 1'b0;
            end
            ALU_MUL: begin
                $display("ALU_MUL\n");
                alu_result = alu_input1 * alu_input2; // 暫時，之後要改成 MULDIV_unit
                alu_branch = 1'b0;
            end
            default: begin
                alu_result = 32'h0;
                alu_branch = 1'b0;
            end
        endcase
        $display("ALU alu_result = %d, alu_branch = %d\n", alu_result, alu_branch);
    end
endmodule

module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];


    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i];
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module MULDIV_unit(
    // TODO: port declaration
    );
    // Todo: HW2
endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
            input i_proc_finish,
            output o_cache_finish,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W*4-1:0]  o_mem_wdata,
            input [BIT_W*4-1:0] i_mem_rdata,
            input i_mem_stall,
            output o_cache_available
        // others
        // input  [ADDR_W-1: 0] i_offset
    );

    assign o_cache_available = 0; // change this value to 1 if the cache is implemented

    //------------------------------------------//
    //          default connection              //
    assign o_mem_cen = i_proc_cen;              //
    assign o_mem_wen = i_proc_wen;              //
    assign o_mem_addr = i_proc_addr;            //
    assign o_mem_wdata = i_proc_wdata;          //
    assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
    assign o_proc_stall = i_mem_stall;          //
    //------------------------------------------//

    // Todo: BONUS

endmodule