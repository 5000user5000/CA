module ALU #(
    parameter DATA_W = 32
)
(
    input                       i_clk,   // clock
    input                       i_rst_n, // reset

    input                       i_valid, // input valid signal
    input [DATA_W - 1 : 0]      i_A,     // input operand A
    input [DATA_W - 1 : 0]      i_B,     // input operand B
    input [         2 : 0]      i_inst,  // instruction

    output [2*DATA_W - 1 : 0]   o_data,  // output value
    output                      o_done   // output valid signal
);
// Do not Modify the above part !!!

// Parameters
    // ======== choose your FSM style ==========
    // 1. FSM based on operation cycles
    // parameter S_IDLE           = 2'd0;
    // parameter S_ONE_CYCLE_OP   = 2'd1;
    // parameter S_MULTI_CYCLE_OP = 2'd2;
    // 2. FSM based on operation modes
    parameter S_IDLE = 4'd0;
    parameter S_ADD  = 4'd1;
    parameter S_SUB  = 4'd2;
    parameter S_AND  = 4'd3;
    parameter S_OR   = 4'd4;
    parameter S_SLT  = 4'd5;
    parameter S_SRA  = 4'd6;
    parameter S_MUL  = 4'd7;
    parameter S_DIV  = 4'd8;
    parameter S_OUT  = 4'd9;

// Wires & Regs
    // Todo
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    // state
    reg  [         3: 0] state, state_nxt; // remember to expand the bit width if you want to add more states!
    // load input
    reg  [  DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  [  DATA_W-1: 0] operand_b, operand_b_nxt;
    reg  [         2: 0] inst, inst_nxt;

// Wire Assignments
    // Todo

    
// Always Combination
    // load input
    always @(*) begin
        if (i_valid) begin
            operand_a_nxt = i_A;
            operand_b_nxt = i_B;
            inst_nxt      = i_inst;
        end
        else begin
            operand_a_nxt = operand_a;
            operand_b_nxt = operand_b;
            inst_nxt      = inst;
        end
    end
    // Todo: FSM
    always @(*) begin
        case(state)
            S_IDLE           : begin
                    if(i_valid == 1'd0) begin
                        state_nxt = S_IDLE;
                    end
                    else begin
                        case(inst_nxt)
                            0: state_nxt = ADD;
                            1: state_nxt = SUB;
                            2: state_nxt = AND;
                            3: state_nxt = OR;
                            4: state_nxt = SLT;
                            5: state_nxt = SRA;
                            6: state_nxt = MUL;
                            7: state_nxt = DIV;
                            default: state_nxt = S_IDLE;
                        endcase
                    end
                end
            ADD : state_nxt = S_IDLE;
            SUB : state_nxt = S_IDLE;
            AND : state_nxt = S_IDLE;
            OR : state_nxt = S_IDLE;
            SLT : state_nxt = S_IDLE;
            SRA : state_nxt = S_IDLE;
            MUL  : state_nxt = S_IDLE;
            DIV  : state_nxt = S_IDLE; // counter == 31 是什麼意思?
            OUT : state_nxt = S_IDLE; 
            default : state_nxt = state;
        endcase
    
    end
    // Todo: Counter
    always @(*) begin
        if(state==MUL || state==DIV)
            counter_nxt = counter + 1;
        else
            counter_nxt = 0;
    end

    // Todo: ALU output
    always @(*) begin
        case(state)
            MUL: begin
                if(shreg[0]==1'b1)
                    alu_out = alu_in;
                else
                    alu_out = 0;
            end
            DIV: begin
                alu_out = shreg[62:31] - alu_in[31:0];
                //since dividend will be shifted for 1 bit left at first
                //but we skip that process , so we use 62:31.
            end
            default: alu_out = 0;
        endcase
    end
    // Todo: output valid signal

    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= S_IDLE;
            operand_a   <= 0;
            operand_b   <= 0;
            inst        <= 0;
        end
        else begin
            state       <= state_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            inst        <= inst_nxt;
        end
    end

endmodule