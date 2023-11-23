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
    // finnish procedure                                                                        //
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
    parameter ALUOP_ECALL = 3'b110; // ????
    parameter ALUOP_DONOTHING = 3'b111; // do nothing

    // alu action
    parameter ALU_ADD = 3'b000;
    parameter ALU_SUB = 3'b001;
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
        wire [BIT_W-1:0] alu_PC_input, alu_input1, alu_input2, alu_result, alu_branch_jump;

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

    // PC_mux_sel_Gen wire connection
    PC_mux_sel_Gen PC_mux_sel_Gen0(
        .jump               (jump),
        .branch             (branch),
        .alu_branch_jump    (alu_branch_jump),
        .sel                (PC_mux_sel)
    );

    // PC_mux wire connection
    Mux3To1 PC_mux(
        .in0    (PC_add_4),
        .in1    (PC_add_imm),
        .in2    (alu_result),
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
        .wdata  (),             // TODO: data memory output
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
            // Where to get pc?
            AUPIC_OPCODE: begin // reg[rd] = pc + {imm, 12'b0};
                jump = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
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
                alu_op = ALUOP_OPETATION; 
            end
            ECALL_OPCODE: begin
                jump = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 0;
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
    output [1:0] sel;

    assign sel = jump ? 2'b10 : (branch && alu_branch_jump) ? 2'b01 : 2'b00; 
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


module Mux3To1 #(
    parameter BIT_W = 32
)(
    input  [BIT_W-1:0]  in0,
    input  [BIT_W-1:0]  in1,
    input  [BIT_W-1:0]  in2,
    input  [1:0]        sel,
    output [BIT_W-1:0]  out

    assign out = (sel == 2'b00) ? in0 : (sel == 2'b01) ? in1 : in2;
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