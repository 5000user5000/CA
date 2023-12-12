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
    // // opcodes
    // parameter AUIPC_OPCODE = 7'b0010111;
    // parameter JAL_OPCODE = 7'b1101111;
    // parameter JALR_OPCODE = 7'b1100111;
    // parameter BRANCH_OPCODE = 7'b1100011;
    // parameter LOAD_OPCODE = 7'b0000011;
    // parameter STORE_OPCODE = 7'b0100011;
    // parameter OP_IMM_OPCODE = 7'b0010011;
    // parameter OP_OPCODE = 7'b0110011;
    // parameter ECALL_OPCODE = 7'b1110011;

    // // alu control input
    // parameter ALUOP_LOAD_STORE = 3'b000; // alu action is add
    // parameter ALUOP_OPETATION = 3'b001; // need check func3 and func7 to know which alu action to use
    // parameter ALUOP_IMM_OPETATION = 3'b010; // need check func3 to know which alu action to use
    // parameter ALUOP_BRANCH = 3'b011;  // need check func3 to know which alu action to use
    // parameter ALUOP_AUIPC = 3'b100; // alu action is add pc
    // parameter ALUOP_JAL_JALR = 3'b101;  // alu action is add pc
    // parameter ALUOP_ECALL = 3'b110; // THE end of program
    // parameter ALUOP_DONOTHING = 3'b111; // do nothing

    // // alu_signal
    // parameter ALU_AND = 4'b0000;
    // parameter ALU_OR  = 4'b0001;
    // parameter ALU_ADD = 4'b0010;
    // parameter ALU_SUB = 4'b0011;
    // parameter ALU_SLL = 4'b0100;
    // parameter ALU_NOR = 4'b0101;
    // parameter ALU_XOR = 4'b0110;
    // parameter ALU_SRL = 4'b0111;
    // parameter ALU_SLT = 4'b1000;
    // parameter ALU_BEQ = 4'b1001;
    // parameter ALU_BNE = 4'b1010;
    // parameter ALU_BLT = 4'b1011;
    // parameter ALU_BGE = 4'b1100;
    // parameter ALU_MUL = 4'b1101;
    // parameter ALU_DONOTHING = 4'b1110;

    //parameter ALU_SLTU = 4'b1111;

    
    // ... other alu action


// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC = 32'h00010000 , next_PC;
        wire mem_cen, mem_wen;
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        wire mem_stall;

        // PC wire
        wire [BIT_W-1:0] PC_add_4, PC_add_imm, alu_result,next_pc_wire;
        wire PC_mux_sel;
        
        // regfile wire
        wire [BIT_W-1:0] read_data1, read_data2;

        // control input wire
        //wire [6:0] opcode;
        // control output wire
        wire  jump,branch, mem_read, mem_write, reg_write, mem_to_reg, alu_src, jalr_signal;
        wire [2:0] alu_op;

        // imm_gen wire
        wire [BIT_W-1:0] imm;

        // alu wire
        wire [BIT_W-1:0]  alu_input1, alu_input2, alu_result; 
        wire alu_branch;
        wire [3:0] alu_signal;

        // adder imm wire
        wire [BIT_W-1:0] pc_imm_input1;

        // data memory wire
        wire [BIT_W-1:0] data_mem_rdata; // read data from data memory
        wire [BIT_W-1:0] write_back,reg_wdata; // write_back" is the output of "write_back_mux (WB)," and "reg_wdata" is the final value written into the regfile

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
    // 非時序，不用 <=


    // assign PC_mux_sel = 0; // default value
    // assign jump = 0; // default value
    // assign branch = 0; // default value
    // assign alu_branch = 0; // default value
    // assign mem_read = 0; // default value   
    // assign mem_write = 0; // default value
    // assign reg_write = 0; // default value
    // assign mem_to_reg = 0; // default value
    // assign alu_src = 0; // default value
    // assign jalr_signal = 0; // default value

    assign o_IMEM_cen = 1'b1; // instruction memory enable

    assign  o_IMEM_addr = PC; // 更新instruction memory address

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------


    //PC_4_adder wire connection
    Adder PC_4_adder(
        .in0    (PC),
        .in1    (32'h4),
        .out    (PC_add_4)
    );

    // pc_imm_input1_mux ，decide pc_imm_input1 value，jalr or others
    Mux2To1 pc_imm_input1_mux(
        .in0    (PC),
        .in1    (read_data1), // jalr
        .sel    (jalr_signal),
        .out    (pc_imm_input1)
    );


    // PC_imm_adder wire connection
    Adder PC_imm_adder(
        .in0    (pc_imm_input1),
        .in1    (imm),
        .out    (PC_add_imm)
    );


    // PC_mux_sel_Gen wire connection，decide pc_mux source
    PC_mux_sel_Gen PC_mux_sel_Gen0(
        .jump               (jump),
        .branch             (branch),
        .alu_branch         (alu_branch),
        .sel                (PC_mux_sel)
    );

    //PC_mux wire connection，decide next_PC value
    Mux2To1 PC_mux(
        .in0    (PC_add_4),
        .in1    (PC_add_imm), // branch、jal。 jalr rd[rs1]+imm，雖說沒用PC，但是PC_imm1 那裏 .in1 就切換到 read_data1
        .sel    (PC_mux_sel), 
        .out    (next_pc_wire)
    );

    // Control wire connection
    Control control0(
        .opcode     (i_IMEM_data[6:0]),
        .i_clk      (i_clk),
        .jump       (jump),
        .branch     (branch),
        .mem_read   (mem_read),
        .mem_write  (mem_write),
        .reg_write  (reg_write),
        .mem_to_reg (mem_to_reg),
        .alu_src    (alu_src),
        .alu_op     (alu_op),
        .jalr_signal(jalr_signal),
        .o_finish   (o_finish)
    );


    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (reg_write),          
        .rs1    (i_IMEM_data[19:15]),                
        .rs2    (i_IMEM_data[24:20]),                
        .rd     (i_IMEM_data[11:7]),                 
        .wdata  (reg_wdata),             // data memory output or PC+4
        .rdata1 (read_data1),           
        .rdata2 (read_data2)
    );



    // alu_control wire connection
    ALU_control alu_control0(
        .i_clk      (i_clk),
        .alu_op     (alu_op),
        .func3      (i_IMEM_data[14:12]),
        .func7      (i_IMEM_data[31:25]),
        .alu_signal (alu_signal)  // signal to alu to tell which alu action to use
    );


    // reg_wdata_mux wire connection (最終寫回regfile的值)
    Mux3To1 reg_wdata_mux(
        .in0    (write_back),
        .in1    (PC_add_4),  // jal、jalr
        .in2    (PC_add_imm), // auipc
        .sel    (i_IMEM_data[6:0]), // rd 值是否要用到pc
        .out    (reg_wdata)
    );

    // alu wire connection
    ALU alu0(
        .i_clk  (i_clk),
        .in0    (read_data1),
        .in1    (alu_input2),
        .alu_signal (alu_signal),
        .alu_result    (alu_result),
        .alu_branch   (alu_branch)
    );

    // Imm_Gen wire connection
    Imm_Gen imm_gen0(
        .i_clk  (i_clk),
        .opcode (i_IMEM_data[6:0]),
        .inst   (i_IMEM_data),
        .imm    (imm)
    );


    // read_data2_imm_mux wire connection
    Mux2To1 read_data2_imm_mux(
        //.i_clk      (i_clk),
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
        .rdata  (data_mem_rdata)
    );


    // write_back wire connection (data memory 旁邊的mux，並非最終寫回去reg的訊號)
    Mux2To1 write_back_mux(
        //.i_clk      (i_clk),
        .in0    (alu_result),
        .in1    (data_mem_rdata),
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
            PC <= next_pc_wire;
        end
       $display("PC = %h, next_PC = %h", PC, next_pc_wire);
       $display("PC_add_4 = %h, PC_add_imm = %h, PC_mux_sel = %h", PC_add_4, PC_add_imm, PC_mux_sel);
       $display("jump = %h , branch = %h , alu_branch = %h , PC_mux_sel = %h", jump, branch, alu_branch, PC_mux_sel);
       $display("imm = %h", imm);
       $display("chip.v i_IMEM_data = %h", i_IMEM_data);
       $display("pc_imm_input1 = %h , read_data1 = %h , jalr_signal = %h", pc_imm_input1, read_data1, jalr_signal);

       $display("alu_signal = %h", alu_signal);
       $display("reg_wdata = %h",  reg_wdata);

       $display("read_data2 = %h, alu_src = %h, alu_input2 = %h", read_data2, alu_src, alu_input2);
       $display("alu_result = %h, data_mem_rdata = %h, mem_to_reg = %h, write_back_mux = %h", alu_result, data_mem_rdata, mem_to_reg, write_back);
        
    end
    

endmodule


module Control (opcode, i_clk ,jump, branch, mem_read, mem_write, reg_write, mem_to_reg, alu_src, jalr_signal, alu_op, o_finish);

    input [6:0] opcode;
    input i_clk;
    output reg  jump,branch, mem_read, mem_write, reg_write, mem_to_reg, alu_src, jalr_signal;
    output reg [2:0] alu_op;
    output reg o_finish; // finish program


    localparam  AUIPC_OPCODE = 7'b0010111;
    localparam  JAL_OPCODE = 7'b1101111;
    localparam  JALR_OPCODE = 7'b1100111;
    localparam  BRANCH_OPCODE = 7'b1100011;
    localparam  LOAD_OPCODE = 7'b0000011;
    localparam  STORE_OPCODE = 7'b0100011;
    localparam  OP_IMM_OPCODE = 7'b0010011;
    localparam  OP_OPCODE = 7'b0110011;
    localparam  ECALL_OPCODE = 7'b1110011;

    // alu control input
    localparam  ALUOP_LOAD_STORE = 3'b000; // alu action is add
    localparam  ALUOP_OPETATION = 3'b001; // need check func3 and func7 to know which alu action to use
    localparam  ALUOP_IMM_OPETATION = 3'b010; // need check func3 to know which alu action to use
    localparam  ALUOP_BRANCH = 3'b011;  // need check func3 to know which alu action to use
    localparam  ALUOP_AUIPC = 3'b100; // alu action is add pc
    localparam  ALUOP_JAL_JALR = 3'b101;  // alu action is add pc
    localparam  ALUOP_ECALL = 3'b110; // THE end of program
    localparam  ALUOP_DONOTHING = 3'b111; // do nothing


    always @(*) begin
        $display("Control opcode = %b", opcode);
        case(opcode)
            AUIPC_OPCODE: begin // reg[rd] = pc + {imm, 12'b0};
                jump = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                jalr_signal = 0;
                alu_op = ALUOP_AUIPC;
                $display("AUIPC\n");
            end
            JAL_OPCODE: begin // reg[rd] = pc + 4; pc = pc + {imm, 12'b0};
                jump = 1; // need jal
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                jalr_signal = 0;
                alu_op = ALUOP_JAL_JALR;
                $display("JAL\n");
            end
            JALR_OPCODE: begin // reg[rd] = pc + 4; pc = reg[rs1] + {imm, 12'b0};
                jump = 1; // need jalr
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                jalr_signal = 1;
                alu_op = ALUOP_JAL_JALR;
                $display("JALR\n");
            end
            BRANCH_OPCODE: begin 
                jump = 0;
                branch = 1; // need branch
                mem_read = 0;
                mem_write = 0;
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 0;
                jalr_signal = 0;
                alu_op = ALUOP_BRANCH;
                $display("BRANCH\n");
            end
            LOAD_OPCODE: begin // reg[rd] = M[reg[rs1] + imm];
                jump = 0;
                branch = 0;
                mem_read = 1; // need to read memory
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 1; // need to use memory data 
                alu_src = 1; // need to use imm
                jalr_signal = 0;
                alu_op = ALUOP_LOAD_STORE;
                $display("LOAD\n");
            end
            STORE_OPCODE: begin // M[reg[rs1] + imm] = reg[rs2];
                jump = 0;
                branch = 0; 
                mem_read = 0;
                mem_write = 1; // need to write memory
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                jalr_signal = 0;
                alu_op = ALUOP_LOAD_STORE;
                $display("STORE\n");
            end
            OP_IMM_OPCODE: begin
                jump = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 1; // need to use imm
                jalr_signal = 0;
                alu_op = ALUOP_IMM_OPETATION;
                $display("OP_IMM\n");
            end
            OP_OPCODE: begin
                jump = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 1; // need to write rd
                mem_to_reg = 0;
                alu_src = 0;
                jalr_signal = 0;
                alu_op = ALUOP_OPETATION; 
                $display("OP\n");
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
                jalr_signal = 0;
                alu_op = ALUOP_ECALL;
                $display("ECALL\n");
            end
            default: begin
                jump = 0;
                branch = 0;
                mem_read = 0;
                mem_write = 0;
                reg_write = 0;
                mem_to_reg = 0;
                alu_src = 0;
                jalr_signal = 0;
                alu_op = ALUOP_DONOTHING;
                $display("default do nothing\n");
            end
        endcase
    end
endmodule

module Imm_Gen #(
    parameter BIT_W = 32,
    parameter AUIPC_OPCODE = 7'b0010111,
    parameter JAL_OPCODE = 7'b1101111,
    parameter JALR_OPCODE = 7'b1100111,
    parameter BRANCH_OPCODE = 7'b1100011,
    parameter LOAD_OPCODE = 7'b0000011,
    parameter STORE_OPCODE = 7'b0100011,
    parameter OP_IMM_OPCODE = 7'b0010011,
    parameter OP_OPCODE = 7'b0110011
)(
    input  [6:0]        opcode,
    input  [BIT_W-1:0]  inst,
    input i_clk,
    output [BIT_W-1:0]  imm
);

    reg [BIT_W-1:0] imm;

    always @(negedge i_clk) begin
        case(opcode)
            AUIPC_OPCODE: begin 
                imm = {inst[31:12], 12'b0};
                $display("AUIPC imm = %d\n", imm);
            end
            JAL_OPCODE: begin 
                imm = {11'b0, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};
                $display("JAL imm = %d\n", imm);
            end
            JALR_OPCODE: begin 
                imm = {20'b0, inst[31:20]};
                $display("JALR imm = %d\n", imm);
            end
            BRANCH_OPCODE: begin 
                imm = {19'b0, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
                $display("BRANCH imm = %d\n", imm);
            end
            LOAD_OPCODE: begin 
                imm = {20'b0, inst[31:20]};
                $display("LOAD imm = %d\n", imm);
            end
            STORE_OPCODE: begin 
                imm = {20'b0, inst[31:25], inst[11:7]};
                $display("STORE imm = %d\n", imm);
            end
            OP_IMM_OPCODE: begin
                imm = {20'b0, inst[31:20]};
                $display("OP_IMM imm = %d\n", imm);
            end
            default: begin
                imm = {32'b0};
                $display("default imm = %d\n", imm);
            end
        endcase
    end
endmodule


module PC_mux_sel_Gen #(
    parameter BIT_W = 32
)(
    input  jump,
    input  branch,
    input  alu_branch,
    output  sel
);
    reg   sel;

    always @(*) begin    
        //sel = jump ? 1'b1 : (branch && alu_branch) ? 1'b1 : 1'b0; // jump = 1，branch = 1，others = 0。adder_imm calculate jump or branch address
        sel = jump; // jump = 1 => jalr, jal , jump = 0 => others
        if(branch == 1 && alu_branch == 1) begin // branch = 1，alu_branch = 1，jump
            sel = 1;
        end
    end
endmodule

module Mux2To1 #(
    parameter BIT_W = 32
)(
    input  [BIT_W-1:0]  in0,
    input  [BIT_W-1:0]  in1,
    input               sel,
    //input i_clk,
    output  [BIT_W-1:0]  out
);
    reg [BIT_W-1:0] out;
    always @(*) begin 
        if(sel == 1'b1) begin
            out = in1;
        end
        else begin
            out = in0;
        end
       //out = sel ? in1 : in0;
       //$display("Mux2To1 in0 = %h, in1 = %h, out = %h", in0, in1, out);
    end
endmodule

module Mux3To1 #(
    parameter BIT_W = 32
)(
    input  [BIT_W-1:0]  in0,
    input  [BIT_W-1:0]  in1,
    input  [BIT_W-1:0]  in2,
    input  [6:0]        sel,
    output [BIT_W-1:0]  out
);
    reg [BIT_W-1:0] out;

    always @(*) begin 
        //out = (sel == 2'b00) ? in0 : (sel == 2'b01) ? in1 : in2;
        if(sel == 7'b0010111) begin // AUIPC
            out = in2;
        end
        else if(sel == 7'b1101111 | sel == 7'b1100111) begin // jal jalr
            out = in1;
        end
        else begin
            out = in0;
        end
    end
endmodule

module Adder #(
    parameter BIT_W = 32
)(
    input  [BIT_W-1:0]  in0,
    input  [BIT_W-1:0]  in1,
    output [BIT_W-1:0]  out
);
    reg [BIT_W-1:0] out;

    always @(*) begin 
        out = in0 + in1;
        //$display("in0 = %d, in1 = %d, out = %d", in0, in1, out);
    end
endmodule

module ALU_control #(
    parameter BIT_W = 32,

    parameter ALUOP_LOAD_STORE = 3'b000, // alu action is add
    parameter ALUOP_OPETATION = 3'b001, // need check func3 and func7 to know which alu action to use
    parameter ALUOP_IMM_OPETATION = 3'b010, // need check func3 to know which alu action to use
    parameter ALUOP_BRANCH = 3'b011,  // need check func3 to know which alu action to use
    parameter ALUOP_AUIPC = 3'b100, // alu action is add pc
    parameter ALUOP_JAL_JALR = 3'b101,  // alu action is add pc
    parameter ALUOP_ECALL = 3'b110, // THE end of program
    parameter ALUOP_DONOTHING = 3'b111, // do nothing

    parameter ALU_AND = 4'b0000,
    parameter ALU_OR  = 4'b0001,
    parameter ALU_ADD = 4'b0010,
    parameter ALU_SUB = 4'b0011,
    parameter ALU_SLL = 4'b0100,
    parameter ALU_NOR = 4'b0101,
    parameter ALU_XOR = 4'b0110,
    parameter ALU_SRL = 4'b0111,
    parameter ALU_SLT = 4'b1000,
    parameter ALU_BEQ = 4'b1001,
    parameter ALU_BNE = 4'b1010,
    parameter ALU_BLT = 4'b1011,
    parameter ALU_BGE = 4'b1100,
    parameter ALU_MUL = 4'b1101,
    parameter ALU_DONOTHING = 4'b1110
)(
    input  [2:0]        alu_op,
    input  [2:0]        func3,
    input  [6:0]        func7,
    input i_clk,
    output [3:0]        alu_signal // signal to alu
);
    reg [3:0] alu_signal;

    always @(*) begin
       //$display("ALU_control alu_op = %b, func3 = %b, func7 = %b", alu_op, func3, func7);
        case(alu_op)
            ALUOP_LOAD_STORE: begin
                alu_signal = ALU_ADD; // instr: ld、sd、jalr。alu action is add
            end
            ALUOP_OPETATION: begin
                if (func7[6]==1'b1) begin
                    alu_signal = ALU_SUB; // instr: R-type sub。alu action is sub
                end
                else if(func7[0]==1'b1) begin
                    alu_signal = ALU_MUL; // instr: R-type ，alu action is MUL
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
                    default: alu_signal = ALU_DONOTHING;
                endcase
                $display("func3 = %b, alu_signal = %b", func3, alu_signal);
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
                    default: alu_signal = ALU_DONOTHING;
                endcase
            end
            ALUOP_BRANCH: begin
                case (func3)
                    3'b000: alu_signal = ALU_BEQ; // beq
                    3'b001: alu_signal = ALU_BNE; // bne
                    3'b100: alu_signal = ALU_BLT; // blt
                    3'b101: alu_signal = ALU_BGE; // bge
                    // 3'b110: alu_signal = ALU_SLT; // bltu，not used
                    // 3'b111: alu_signal = ALU_SRL; // bgeu
                    default: alu_signal = ALU_DONOTHING;
                endcase
            end
            ALUOP_AUIPC: begin
                alu_signal = ALU_ADD; // instr: auipc。alu action is add
            end
            ALUOP_JAL_JALR: begin
                alu_signal = ALU_DONOTHING; // instr: jal。alu action is do nothing
            end
            ALUOP_ECALL: begin
                alu_signal = ALU_DONOTHING; // instr: ecall。default alu action is do nothing
            end
            default: begin
                alu_signal = ALU_DONOTHING; // JUST a default value
            end
        endcase
    end
endmodule

module ALU #(
    parameter BIT_W = 32,

    parameter ALU_AND = 4'b0000,
    parameter ALU_OR  = 4'b0001,
    parameter ALU_ADD = 4'b0010,
    parameter ALU_SUB = 4'b0011,
    parameter ALU_SLL = 4'b0100,
    parameter ALU_NOR = 4'b0101,
    parameter ALU_XOR = 4'b0110,
    parameter ALU_SRL = 4'b0111,
    parameter ALU_SLT = 4'b1000,
    parameter ALU_BEQ = 4'b1001,
    parameter ALU_BNE = 4'b1010,
    parameter ALU_BLT = 4'b1011,
    parameter ALU_BGE = 4'b1100,
    parameter ALU_MUL = 4'b1101,
    parameter ALU_DONOTHING = 4'b1110
)(
    input  [BIT_W-1:0]  in0,
    input  [BIT_W-1:0]  in1,
    input  [3:0]        alu_signal,
    input i_clk,
    output [BIT_W-1:0]  alu_result,
    output      alu_branch // alu branch  signal
);
    reg alu_branch;
    reg [BIT_W-1:0] alu_result;

    always @(*) begin
        //$display("ALU in0 = %d, in1 = %d, alu_signal = %b", in0, in1, alu_signal);
        case(alu_signal)
            ALU_ADD: begin
                alu_result = in0 + in1; // 先不想overflow的問題
                alu_branch = 0;
                $display("ALU_ADD   in0 = %d, in1 = %d, alu result = %d\n", in0, in1, alu_result);
            end
            ALU_SUB: begin
                alu_result = in0 - in1;
                alu_branch = 0;
                $display("ALU_SUB, alu result = %d\n", alu_result);
            end
            ALU_AND: begin
                alu_result = in0 & in1;
                alu_branch = 0;
            end
            ALU_OR: begin
                alu_result = in0 | in1;
                alu_branch = 0;
            end
            ALU_XOR: begin
                alu_result = in0 ^ in1;
                alu_branch = 0;
            end
            ALU_SLL: begin
                alu_result = in0 << in1;
                alu_branch = 0;
            end
            ALU_SRL: begin
                alu_result = in0 >> in1;
                alu_branch = 0;
            end
            ALU_SLT: begin
                alu_result = (in0 < in1) ? 1 : 0;
                alu_branch = 0;
            end
            ALU_NOR: begin
                alu_result = ~(in0 | in1);
                alu_branch = 0;
            end
            ALU_BEQ: begin
                alu_result = 0;
                alu_branch = (in0 == in1) ? 1 : 0; 
            end
            ALU_BNE: begin
                alu_result = 0;
                alu_branch = (in0 != in1) ? 1 : 0; 
            end
            ALU_BLT: begin
                alu_result = 0;
                alu_branch = (in0 < in1) ? 1 : 0; 
            end
            ALU_BGE: begin
                alu_result = 0;
                alu_branch = (in0 >= in1) ? 1 : 0; 
            end
            ALU_MUL: begin
                alu_result = in0 * in1; // 暫時，之後要改成 MULDIV_unit
                alu_branch = 0;
            end
            default: begin
                alu_result = in0 + in1;
                alu_branch = 0;
            end
        endcase
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
        //$display("      rs1 = %d, rs2 = %d, rdata1 = %d, rdata2 = %d \n", rs1, rs2, rdata1, rdata2);
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
    parameter BIT_W = 32,
    parameter BITS = 32,
    parameter word_depth = 32
)(  
    input [BIT_W-1:0] addr,
    input [BIT_W-1:0] wdata,
    input wen, // wen: 0:read | 1:write
    output [BIT_W-1:0] rdata
);
    reg [BITS-1:0] mem [0:word_depth-1]; // 32 32-bit memory
    reg [BITS-1:0] rdata;
    
    always @(*) begin 
        if(wen) begin
            mem[addr] = wdata;
            rdata = 0; // default value
        end
        else begin
            rdata = mem[addr];
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
            output o_cache_available,
        // others
        input  [ADDR_W-1: 0] i_offset
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