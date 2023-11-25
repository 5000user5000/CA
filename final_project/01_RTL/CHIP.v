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
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any declaration

    // opcodes
    parameter AUIPC_OPCODE = 7'b0010111;
    parameter JAL_OPCODE = 7'b1101111;
    parameter JALR_OPCODE = 7'b1100111;
    parameter BRANCH_OPCODE = 7'b1100011;
    parameter LOAD_OPCODE = 7'b0000011;
    parameter STORE_OPCODE = 7'b0100011;
    parameter OP_IMM_OPCODE = 7'b0010011;
    parameter OP_OPCODE = 7'b0110011;
    parameter ECALL_OPCODE = 7'b1110011;

    // alu control input
    parameter ALUOP_LOAD_STORE_JALR = 3'b000; // alu action is add
    parameter ALUOP_OPETATION = 3'b001; // need check func3 and func7 to know which alu action to use
    parameter ALUOP_IMM_OPETATION = 3'b010; // need check func3 to know which alu action to use
    parameter ALUOP_BRANCH = 3'b011;  // need check func3 to know which alu action to use
    parameter ALUOP_AUIPC = 3'100; // alu action is add pc
    parameter ALUOP_JAL = 3'b101;  // alu action is add pc
    parameter ALUOP_ECALL = 3'b110; // THE end of program
    parameter ALUOP_DONOTHING = 3'b111; // do nothing

    // alu action (alu_signal)
    parameter ALU_AND = 4'b0000;
    parameter ALU_OR  = 4'b0001;
    parameter ALU_ADD = 4'b0010;
    parameter ALU_SUB = 4'b0110;
    parameter ALU_SLL = 4'b0111;
    parameter ALU_NOR = 4'b1100;
    parameter ALU_XOR = 4'b1101;
    parameter ALU_SRL = 4'b1110;
    parameter ALU_SLT = 4'b1111;
    //parameter ALU_SLTU = 4'b1111;

    
    // ... other alu action


// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, next_PC;
        wire mem_cen, mem_wen;
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        wire mem_stall;

        // PC wire
        wire [BIT_W-1:0] PC_add_4, PC_add_imm, alu_result;
        wire [1:0] PC_mux_sel;
        
        // regfile wire
        wire [BIT_W-1:0] read_data1, read_data2;

        // control input wire
        wire [6:0] opcode;
        // control output wire
        wire jump, branch, mem_read, mem_write, reg_write, mem_to_reg, alu_src;
        wire [2:0] alu_op;

        // imm_gen wire
        wire [BIT_W-1:0] imm;

        // alu wire
        wire [BIT_W-1:0] alu_signal, alu_input1, alu_input2, alu_result, alu_branch_jump, alu_with_pc; // alu_with_pc such as jalr, jal, auipc need to use pc as input1

        // data memory wire
        wire [BIT_W-1:0] data_mem_rdata; // read data from data memory
        wire [BIT_W-1:0] write_back; // write back to regfile

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // PC_4_adder wire connection
    Adder PC_4_adder(
        .in0    (PC),
        .in1    (32'h4),
        .out    (PC_add_4)
    );

    // PC_imm_adder wire connection
    Adder PC_imm_adder(
        .in0    (PC),
        .in1    (imm),
        .out    (PC_add_imm)
    );

    // PC_mux_sel_Gen wire connection，decide pc_mux source
    PC_mux_sel_Gen PC_mux_sel_Gen0(
        .jump               (jump),
        .branch             (branch),
        .alu_branch_jump    (alu_branch_jump),
        .sel                (PC_mux_sel)
    );

    // PC_mux wire connection，decide next_PC value
    Mux2To1 PC_mux(
        .in0    (PC_add_4),
        .in1    (PC_add_imm), // jump or branch
        .sel    (PC_mux_sel),
        .out    (next_PC)
    );


    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (reg_write),          
        .rs1    (i_IMEM_data[19:15]),                
        .rs2    (i_IMEM_data[24:20]),                
        .rd     (i_IMEM_data[11:7]),                 
        .wdata  (write_back),             // data memory output
        .rdata1 (read_data1),           
        .rdata2 (read_data2)
    );

    // Control wire connection
    Control control0(
        .opcode     (opcode),
        .jump       (jump),
        .branch     (branch),
        .mem_read   (mem_read),
        .mem_write  (mem_write),
        .reg_write  (reg_write),
        .mem_to_reg (mem_to_reg),
        .alu_src    (alu_src),
        .alu_op     (alu_op)
    );

    // alu_control wire connection
    ALU_control alu_control0(
        .alu_op     (alu_op),
        .func3      (i_IMEM_data[14:12]),
        .func7      (i_IMEM_data[31:25]),
        .alu_signal (alu_signal)  // signal to alu to tell which alu action to use
    );

    // alu_input1_mux wire connection
    Mux2To1 alu_input1_mux(
        .in0    (read_data1),
        .in1    (PC),
        .sel    (alu_with_pc), 
        .out    (alu_input1)
    );

    // alu wire connection
    ALU alu0(
        .in0    (alu_input1),
        .in1    (alu_input2),
        .alu_signal (alu_signal),
        .out    (alu_result),
        .zero   (alu_branch_jump)
    );

    // Imm_Gen wire connection
    Imm_Gen imm_gen0(
        .opcode (opcode),
        .inst   (i_IMEM_data),
        .imm    (imm)
    );

    // read_data2_imm_mux wire connection
    Mux2To1 read_data2_imm_mux(
        .in0    (read_data2),
        .in1    (imm),
        .sel    (alu_src),
        .out    (alu_input2)
    );

    // data_memory wire connection
    data_memory data_memory0(
        .addr   (alu_result),
        .wdata  (read_data2),
        .wen    (mem_write),
        .cen    (mem_cen),
        .rdata  (data_mem_rdata)
    );

    // write_back wire connection
    Mux2To1 write_back_mux(
        .in0    (data_mem_rdata),
        .in1    (alu_result),
        .sel    (mem_to_reg),
        .out    (write_back)
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
            PC <= next_PC;
        end
    end
endmodule


module Control(
    input [6:0] opcode,
    output jump, branch, mem_read, mem_write, reg_write, mem_to_reg, alu_src,
    output [2:0] alu_op

    always @(posedge clk) begin
        case(opcode)
            AUPIC_OPCODE: begin // reg[rd] = pc + {imm, 12'b0};
                jump = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                alu_with_pc = 1; // need to use pc
                alu_op = ALUOP_AUIPC;
            end
            JAL_OPCODE: begin // reg[rd] = pc + 4; pc = pc + {imm, 12'b0};
                jump = 1; // need jump
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                alu_with_pc = 1; // need to use pc
                alu_op = ALUOP_JAL;
            end
            JALR_OPCODE: begin // reg[rd] = pc + 4; pc = reg[rs1] + {imm, 12'b0};
                jump = 1; // need jump
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                alu_with_pc = 1; // need to use pc
                alu_op = ALUOP_LOAD_STORE_JALR;
            end
            BRANCH_OPCODE: begin 
                jump = 0;
                branch = 1; // need branch
                mem_read = 0;
                mem_write = 0;
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 0;
                alu_with_pc = 0;
                alu_op = ALUOP_BRANCH;
            end
            LOAD_OPCODE: begin // reg[rd] = M[reg[rs1] + imm];
                jump = 0;
                branch = 0;
                mem_read = 1; // need to read memory
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 1; // need to use memory data 
                alu_src = 1; // need to use imm
                alu_with_pc = 0;
                alu_op = ALUOP_LOAD_STORE_JALR;
            end
            STORE_OPCODE: begin // M[reg[rs1] + imm] = reg[rs2];
                jump = 0;
                branch = 0; 
                mem_read = 0;
                mem_write = 1; // need to write memory
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                alu_with_pc = 0;
                alu_op = ALUOP_LOAD_STORE_JALR;
            end
            OP_IMM_OPCODE: begin
                jump = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                alu_with_pc = 0;
                alu_op = ALUOP_IMM_OPETATION;
            end
            OP_OPCODE: begin
                jump = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 0;
                alu_with_pc = 0;
                alu_op = ALUOP_OPETATION; 
            end
            ECALL_OPCODE: begin
                o_finish = 1; // finish program
                jump = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 0;
                alu_with_pc = 0;
                alu_op = ALUOP_ECALL;
            end
            default: begin
                jump = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 0;
                alu_with_pc = 0;
                alu_op = ALUOP_DONOTHING;
            end
        endcase
    end
);
endmodule

module Imm_Gen #(
    parameter BIT_W = 32
)(
    input  [6:0]        opcode,
    input  [BIT_W-1:0]  inst,
    output [BIT_W-1:0]  imm

    always @(posedge clk) begin
        case(opcode)
            AUPIC_OPCODE: begin 
                imm = {inst[31:12], 12'b0};
            end
            JAL_OPCODE: begin 
                imm = {11'b0, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};
            end
            JALR_OPCODE: begin 
                imm = {20'b0, inst[31:20]};
            end
            BRANCH_OPCODE: begin 
                imm = {19'b0, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
            end
            LOAD_OPCODE: begin 
                imm = {20'b0, inst[31:20]};
            end
            STORE_OPCODE: begin 
                imm = {20'b0, inst[31:25], inst[11:7]};
            end
            OP_IMM_OPCODE: begin
                imm = {20'b0, inst[31:20]};
            end
            default: begin
                imm = {32'b0};
            end
        endcase
    end
);
endmodule


module PC_mux_sel_Gen #(
    parameter BIT_W = 32
)(
    input  jump;
    input  branch;
    input  alu_branch_jump;
    output  sel;

    assign sel = jump ? 1'b1 : (branch && alu_branch_jump) ? 1'b1 : 2'b00; // jump = 1，branch = 1，others = 0。因為上方的加法器 add sum 都會是跳轉的位址
)
endmodule

module Mux2To1 #(
    parameter BIT_W = 32
)(
    input  [BIT_W-1:0]  in0,
    input  [BIT_W-1:0]  in1,
    input               sel,
    output [BIT_W-1:0]  out

    assign out = sel ? in1 : in0;
);
endmodule

module Adder #(
    parameter BIT_W = 32
)(
    input  [BIT_W-1:0]  in0,
    input  [BIT_W-1:0]  in1,
    output [BIT_W-1:0]  out

    assign out = in0 + in1;
);
endmodule

module ALU_control #(
    parameter BIT_W = 32
)(
    input  [2:0]        alu_op,
    input  [2:0]        func3,
    input  [6:0]        func7,
    output [3:0]        alu_signal // signal to alu

    always @(posedge clk) begin
        case(alu_op)
            ALUOP_LOAD_STORE_JALR: begin
                alu_signal = ALU_ADD; // instr: ld、sd、jalr。alu action is add
            end
            ALUOP_OPETATION: begin
                if (func7[6]==1'b1) begin
                    alu_signal = ALU_SUB; // instr: R-type sub。alu action is sub
                end
                else begin
                    case(func3)
                    3'b000: alu_signal = ALU_ADD;
                    3'b001: alu_signal = ALU_SLL;
                    3'b010: alu_signal = ALU_SLT;
                    3'b011: alu_signal = ALU_SLT; // sltu
                    3'b100: alu_signal = ALU_XOR;
                    3'b101: alu_signal = ALU_SRL;
                    3'b110: alu_signal = ALU_OR;
                    3'b111: alu_signal = ALU_AND;
                    default: alu_signal = ALU_ADD;
                endcase
                end
            end
            ALUOP_IMM_OPETATION: begin
                case(func3)
                    3'b000: alu_signal = ALU_ADD;
                    3'b001: alu_signal = ALU_SLL;
                    3'b010: alu_signal = ALU_SLT;
                    3'b011: alu_signal = ALU_SLT; // sltu
                    3'b100: alu_signal = ALU_XOR;
                    3'b101: alu_signal = ALU_SRL;
                    3'b110: alu_signal = ALU_OR;
                    3'b111: alu_signal = ALU_AND;
                    default: alu_signal = ALU_ADD;
                endcase
            end
            ALUOP_BRANCH: begin
                case (func3)
                    3'b000: alu_signal = ALU_AND; // beq
                    3'b001: alu_signal = ALU_OR; // bne
                    3'b100: alu_signal = ALU_SLT; // blt
                    3'b101: alu_signal = ALU_SUB; // bge
                    // 3'b110: alu_signal = ALU_SLT; // bltu，not used
                    // 3'b111: alu_signal = ALU_SRL; // bgeu
                    default: alu_signal = ALU_ADD; // default alu action is add
                endcase
            end
            ALUOP_AUIPC: begin
                alu_signal = ALU_ADD; // instr: auipc。alu action is add
            end
            ALUOP_JAL: begin
                alu_signal = ALU_ADD; // instr: jal。alu action is add
            end
            ALUOP_ECALL: begin
                alu_signal = ALU_ADD; // instr: ecall。default alu action is add
            end
            default: begin
                alu_signal = ALU_ADD; // JUST a default value
            end
        endcase
    end
);
endmodule

module ALU #(
    parameter BIT_W = 32
)(
    input  [BIT_W-1:0]  in0,
    input  [BIT_W-1:0]  in1,
    input  [3:0]        alu_signal,
    output [BIT_W-1:0]  alu_result,
    output      zero,

    always @(posedge clk) begin
        case(alu_signal)
            ALU_ADD: begin
                alu_result = in0 + in1;
            end
            ALU_SUB: begin
                alu_result = in0 - in1;
                zero = (alu_result >= 0) ? 1 : 0; // branch signal，alu_result > 0 -> bge
            end
            ALU_AND: begin
                alu_result = in0 & in1;
                //zero = (alu_result == 0) ? 1 : 0; // branch signal，beq。 wrong because 0000 & 0000 = 0，but should be true
                zero = (in0 == in1) ? 1 : 0; // branch signal，beq
            end
            ALU_OR: begin
                alu_result = in0 | in1;
                zero = (in0 != in1) ? 1 : 0; // branch signal，bne
            end
            ALU_XOR: begin
                alu_result = in0 ^ in1;
            end
            ALU_SLL: begin
                alu_result = in0 << in1;
            end
            ALU_SRL: begin
                alu_result = in0 >> in1;
            end
            ALU_SLT: begin
                alu_result = (in0 < in1) ? 1 : 0;
                zero = (alu_result == 1) ? 1 : 0; // branch signal，blt
            end
            ALU_NOR: begin
                alu_result = ~(in0 | in1);
            end
            default: begin
                alu_result = in0 + in1;
            end
        endcase
    end
);
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

module data_memory #(
    parameter BIT_W = 32
    parameter BITS = 32;
    parameter word_depth = 32;
)(  
    input [BIT_W-1:0] addr,
    input [BIT_W-1:0] wdata,
    input wen, // wen: 0:read | 1:write
    input cen, // cen: 0:inactive | 1:active (enable memory function?) 
    output [BIT_W-1:0] rdata,

    reg [BITS-1:0] mem [0:word_depth-1]; // 32 32-bit memory

    if(cen) begin
        if(wen) begin
            mem[addr] = wdata;
            rdata = 0; // default value
        end
        else begin
            rdata = mem[addr];
        end
    end
    else begin
        rdata = 0;
    end

);
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