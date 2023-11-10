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
    parameter S_IDLE           = 2'd0;
    parameter S_ONE_CYCLE_OP   = 2'd1; 
    parameter S_MULTI_CYCLE_OP = 2'd2; // ex: MUL，DIV
    // 2. FSM based on operation modes
    // parameter S_IDLE = 4'd0;
    // parameter S_ADD  = 4'd1;
    // parameter S_SUB  = 4'd2;
    // parameter S_AND  = 4'd3;
    // parameter S_OR   = 4'd4;
    // parameter S_SLT  = 4'd5;
    // parameter S_SRA  = 4'd6;
    // parameter S_MUL  = 4'd7;
    // parameter S_DIV  = 4'd8;
    // parameter S_OUT  = 4'd9;

    // ======== operation modes ==========
    parameter ADD = 3'd0;
    parameter SUB = 3'd1;
    parameter AND = 3'd2;
    parameter OR  = 3'd3;
    parameter SLT = 3'd4;
    parameter SRA = 3'd5;
    parameter MUL = 3'd6;
    parameter DIV = 3'd7;


// Wires & Regs
    // Todo
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg [31 : 0] alu_out;
    reg [63:0] o_data;
    reg o_done;
    // state
    reg  [         1: 0] state, state_nxt; // remember to expand the bit width if you want to add more states!
    // load input
    reg  signed [  DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  signed [  DATA_W-1: 0] operand_b, operand_b_nxt;
    reg         [         2: 0] inst, inst_nxt;

// Wire Assignments
    // Todo
    always @(posedge i_clk) begin // clk 上升沿
        //$display("result assign， state = %d",state);
        if(state == S_MULTI_CYCLE_OP) begin
            o_data = shreg_nxt;
        end
        else begin
            if(state == S_ONE_CYCLE_OP)
                o_data = {32'b0,alu_out[31:0]};
            else 
                o_data = 64'h777; // fun
        end
        $display("o_data=%h",o_data);
    end

    
// Always Combination
    // load input
    always @(*) begin
        //$display("i_valid=%d",i_valid);
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
        //$display("state=%d",state);
        case(state)
            S_IDLE           : begin
                    if(i_valid == 1'd0) begin
                        state_nxt = S_IDLE;
                    end
                    else begin
                        case(inst_nxt)
                            DIV: state_nxt = S_MULTI_CYCLE_OP;
                            MUL: state_nxt = S_MULTI_CYCLE_OP;
                            default: state_nxt = S_ONE_CYCLE_OP; //除了DIV和MUL以外的都是一個cycle就可以做完
                        endcase
                    end
                end
            S_ONE_CYCLE_OP : state_nxt = S_IDLE;
            S_MULTI_CYCLE_OP : begin
                if(counter != 5'd31) // counter == 31 應該是只要重複作 31+1次
                    state_nxt = S_MULTI_CYCLE_OP;
                else
                    state_nxt = S_IDLE;
            end

            default : state_nxt = state;
        endcase
    
    end
    // Todo: Counter
    always @(negedge i_clk) begin //negedge i_clk
        $display("counter =%d",counter);
        if(state == S_MULTI_CYCLE_OP)
            counter_nxt = counter + 1;
        else 
            counter_nxt = 0;
    end

    // Todo: ALU output
    always @(*) begin
        //$display("inst=%d",inst);
        case(i_inst)
            ADD: begin
                // 處理 overflow
                alu_out[31:0] = operand_a + operand_b;
                if(operand_a[31]==operand_b[31] && operand_a[31]==1'b0  && alu_out[31]==1'b1) begin // 正+正=負 overflow
                    //$display("pos+pos=neg overflow %b +  %b =%b",operand_a,operand_b,alu_out);
                    alu_out[31:0] = 32'h7fffffff; // 2^32-1
                end
                else  if(operand_a[31]==operand_b[31] && operand_a[31]==1'b1 && alu_out[31]==1'b0 )  begin // 負+負=正 overflow
                        //$display("neg+neg=pos overflow %b +  %b =%b",operand_a,operand_b,alu_out);
                        alu_out[31:0] = {1'b1, 31'b0}; // -2^32
                    end
                else
                    $display("no overflow");
                    //$display("no overflow，%b +  %b =%b",operand_a,operand_b,alu_out);
                    //alu_out[31:0] = operand_a + operand_b;
                $display("alu_out = %h",alu_out);
                end
            SUB: begin
                // 處理 overflow
                alu_out[31:0] = operand_a - operand_b;
                if(operand_a[31]!=operand_b[31] && operand_a[31]==1'b0  && alu_out[31]==1'b1) begin // 正-負=負 overflow
                    //$display("pos-neg=neg overflow %b -  %b =%b",operand_a,operand_b,alu_out);
                    alu_out[31:0] = 32'h7fffffff; // 2^32-1
                end
                else if(operand_a[31]!=operand_b[31] && operand_a[31]==1'b1 && alu_out[31]==1'b0 ) begin // 負-正=正 overflow
                    //$display("neg-pos=pos overflow %b -  %b =%b",operand_a,operand_b,alu_out);
                    alu_out[31:0] = {1'b1, 31'b0}; // -2^32
                end
                else
                    $display("no overflow");
                    //$display("no overflow，%b -  %b =%b",operand_a,operand_b,alu_out);
                    //alu_out[31:0] = operand_a - operand_b;
                
            end
            AND: begin
                alu_out[31:0] = operand_a & operand_b;
                //$display("and %b &  %b =%b",operand_a,operand_b,alu_out);
            end
            OR: begin
                alu_out[31:0] = operand_a | operand_b;
                //$display("or %b |  %b =%b",operand_a,operand_b,alu_out);
            end
            SLT: begin
                alu_out[31:0] = (operand_a < operand_b) ? 32'b1 : 32'b0;
                //$display("slt %b <  %b =%b",operand_a,operand_b,alu_out);
            end
            SRA: begin
                alu_out[31:0] = operand_a >>> operand_b; //算數右移(>>>)，不是邏輯右移(>>)
                //$display("sra %b >>>  %b =%b",operand_a,operand_b,alu_out);
            end
            MUL: begin
                if(counter==5'd0) begin// initialize
                    shreg[63:0] = { 32'b0 , operand_b };
                    $display("init");
                end
                if(shreg[0]==1'b1)
                    alu_out = operand_a;// 如果reg最右邊是1，那就把operand_a放到alu_out，之後會加到shreg最左邊
                else
                    alu_out = 0;
            end
            DIV: begin
                alu_out = shreg[62:31] - operand_b[31:0]; // divisor
            end
            default: alu_out = 0;
        endcase
    end

    //shift register
     always @(*) begin
        //$display("shreg，instr=%b",i_inst);
        case(i_inst)
            MUL: begin
                shreg_nxt = {shreg[63:32]+alu_out,shreg[31:0]} >> 1; // multiplier 放在最左邊，右邊是multiplicand，加完後右移一位
                if (shreg_nxt[62:31] < shreg[63:32]) // divisor < remainder
                    shreg_nxt[63] = 1;
                else 
                    shreg_nxt[63] = 0;

            end
            DIV: begin
                if(alu_out[31]==1'b1) begin  //如果是負數
                    shreg_nxt = shreg << 1; 
                    shreg_nxt[0] = 0;
                end
                else begin
                    shreg_nxt = {alu_out[31:0],shreg[31:0]}<<1; 
                    shreg_nxt[0] = 1;
                end
            end
            default: begin
                if(i_valid==1'd1)
                    shreg_nxt = {shreg[63:32], alu_out[31:0]}; 
                else
                    shreg_nxt = 0;
            end
        endcase
    end


    // Todo: output valid signal
    always @(posedge i_clk) begin
        if(state!=S_IDLE && state_nxt== S_IDLE) begin// state_nxt== S_IDLE 代表下一個cycle就會變成S_IDLE，所以這個cycle就是最後一個cycle
            //$display("o_done=1");
            o_done = 1'b1;
        end
        else
            o_done = 1'b0;
    end


    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin // 在 clk 下降沿 同步輸入
        if (!i_rst_n) begin
            //$display("reset!");
            state       <= S_IDLE;
            // operand_a   <= 0;
            // operand_b   <= 0;
            // inst        <= 0;
        end
        else begin
            //$display("update!");
            state       <= state_nxt;
            counter     <= counter_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            inst        <= inst_nxt;
            shreg       <= shreg_nxt;
        end
    end

endmodule
