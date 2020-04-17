`include "cpu.svh"

module cpu
    (
        // since this project is now simulation only you can ignore
        // clk_100M
        input logic clk_100M, clk, rst,
        input logic [31:0] r_data,
        output logic wr_en,
        output logic [31:0] mem_addr, w_data,
        // debug addr/data and instr are for showing CPU information
        // on the FPGA (only useful in synthesis)
        input logic [4:0] rdbg_addr,
        output logic [31:0] rdbg_data,
        output logic [31:0] instr
    );
    
    // first: wire up the circuit (controls can be 0 for now)
    
    // control variables for the datapath
    logic IorD, IRWrite, RegDst, MemtoReg, RegWrite, ALUSrcA, PCSrc, Branch, PCWrite;
    logic [1:0] ALUSrcB;
    
    // this will be diff from the book
    logic [3:0] ALUControl;
    
    // temporary shits for now
    logic [31:0] temp1, temp2, temp3, temp4, temp5, temp6;
    
    // save vals for first two reg's
    reg_en reg_1 (.clk(clk), .rst, .en(IRWrite), .d(r_data), .q(temp1));  
    reg_reset reg_2 (.clk, .rst, .d(r_data), .q(temp2));
    
    logic [4:0] a0, a1, adbg, dest;
    
    // choose which section is the destination (r vs. i/j)
    assign dest = RegDst ? r_data[15:11] : r_data[20:16];
    
    assign a0 = r_data[25:21];
    assign a1 = r_data[20:16];
    
    logic [31:0] r0, r1, rdbg;
    
    // chose b/w ALUOut and memory output (r vs. i/j)
    logic [31:0] w_data;
    assign w_data = MemtoReg ? r_data : alu_out;
    
    // interact w/ register file
    reg_file reg_main (.clk, .wr_en(RegWrite), .w_addr(dest), .r0_addr(a0), .r1_addr(a1), .w_data, .r0_data(r0), .r1_data(r1), .rbdg_addr(adbg), .rbdg_data(rdbg));
    
    // build results from FF following register file
    reg_reset reg_3 (.clk, .rst, .d(r0), .q(temp3));
    reg_reset reg_4 (.clk, .rst, .d(r1), .q(temp4));
    
    // define ALU SrcA
    logic [31:0] SrcA;
    assign SrcA = ALUSrcA ? temp3 : pc_stage;
    
    // sign extend
    logic [31:0] r_ext ;
    assign r_ext = r_data[15] ? {16'b1111111111111111, r_data[15:0]} : {16'b0000000000000000, r_data[15:0]};
    
    // shift extension 2 spots
    logic [31:0] r_sft;
    assign r_sft = r_ext << 2;
    
    // build srcb multiplexer
    logic [31:0] SrcB;
    assign SrcB = ALUSrcB[1] ? (ALUSrcB[0] ? r_sft : r_ext) : (ALUSrcB[0] ? 32'b100 : temp4);

    // interact with the ALU (adjust op codes for larger inst set)
    logic [31:0] alu_res;
    logic zero;
    alu alu_main (.x(SrcA), .y(SrcB), .op(ALUControl), .z(alu_res), .zero);
    
    // control zero and branching
    logic b_temp, PC_en;
    assign b_temp = zero && Branch;
    assign PC_en = b_temp || PCWrite;
    
    // another reg for ALU output
    logic [31:0] alu_out;
    reg_reset reg_5 (.clk, .rst, .d(alu_res), .q(alu_out));
    
    // multiplex on ALU result
    logic [31:0] pc_nxt;
    assign pc_nxt = PCSrc ? alu_out : alu_res;
    
    
    // final register for the PC enabling
    logic [31:0] pc_stage;
    reg_en reg_6 (.clk, .rst, .en(PC_en), .d(pc_nxt), .q(pc_stage));
    
    
    // multiplex on PC choice
    logic [31:0] address;
    assign address = IorD ? alu_out : pc_stage;
    
    // save outputs for memory interface
    assign mem_addr = address;
    assign w_data = temp4;
    
     // second: the FSM... design and build the state changes w/ the appr control vals 
     logic [5:0] state, next_state;
     always_ff @(posedge clk, posedge rst) begin
        if (rst) begin
            state <= `S0;
            // sample_reg <= 0;
        end
        else begin
            state <= next_state;   
        end
    end
    
    always_comb begin
        case (state)
            `S0: begin
                // set new controls
                IorD = 0;
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b01; 
                ALUOp = 2'b00; 
                PCSrc = 2'b00;
                IRWrite = 1'b1;
                PCWrite = 1'b1;
                next_state = `S1;
            end
            `S1: begin
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b11;
                ALUOp = 2'b00;
                if (r_data[31:26] == `OP_RTYPE) begin
                    next_state = `S6;
                end
                else 
                    if (r_data[31:26] == (`OP_LW || `OP_SW)) begin
                        next_state = `S2;
                    end
                    else 
                        if (r_data[31:26] == `OP_BEQ) begin
                            next_state = `S8;
                        end
                        else
                            if (r_data[31:26] == `OP_ADDI) begin
                                next_state = `S9;
                            end
                            else
                                if (r_data[31:26] == `OP_J) begin
                                    next_state = `S11;
                                end
           end                
           `S2: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b10;
                ALUOp = 2'b11;
                if (r_data[31:26] == `OP_LW) begin
                    next_state = `S3;
                end
                else
                    if (r_data[31:25] == `OP_SW) begin
                        next_state = `S5;
                    end
           end
           `S3: begin
                IorD = 1'b1;
                next_state = `S4;
           end
           `S4: begin
                RegDst = 1'b0;
                MemtoReg = 1'b1;
                RegWrite = 1'b1;
           end
           `S5: begin
                IorD = 1'b1;
                wr_en = 1'b1;
                next_state = `S0;
           end
           `S6: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b00;
                ALUOp = 2'b10;
                next_state = `S7;
           end
           `S7: begin
                RegDst = 1'b1;
                MemtoReg = 1'b0;
                RegWrite = 1'b1;
                next_state = `S0;
           end
           `S8: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b00;
                ALUOp = 2'b01;
                PCSrc = 2'b01;
                Branch = 1'b1;
                next_state = `S0;
           end
           `S9: begin
            
           end
                

                saver = `NS_GREEN;
                next_state = `LOADER;
                
                // set next value for internal registers
                // sample_reg_next = 1; 
            end
            `NS_GREEN: begin
                light_ns = `LIGHT_GREEN;
                light_ew = `LIGHT_RED;
                light_ped = `PED_NS; 
                timer_en = 1; 
                timer_load = 0; 

                saver = `NS_YELLOW;
                next_state = `LOADER;
                
                // set next value for internal registers
                // sample_reg_next = 1; 
            end
            `NS_YELLOW: begin
                light_ns = `LIGHT_YELLOW;
                light_ew = `LIGHT_RED;
                light_ped = `PED_NS; 
                timer_en = 1; 
                timer_load = 0; 

                if (ped)
                    saver = `PED_2;
                else
                    saver = `EW_GREEN;
                
                next_state = `LOADER;
                
                // set next value for internal registers
                // sample_reg_next = 1; 
            end
            `PED_2: begin
                light_ns = `LIGHT_RED;
                light_ew = `LIGHT_RED;
                light_ped = `PED_BOTH; 
                timer_en = 1; 
                timer_load = 0; 

                if (car_ns)
                    if (car_ew)
                        saver = `EW_GREEN;
                    else
                        saver = `NS_GREEN;
                else
                    saver = `EW_GREEN;
                
                next_state = `LOADER;
                
                // set next value for internal registers
                // sample_reg_next = 1; 
            end
            `EW_GREEN: begin
                light_ns = `LIGHT_RED;
                light_ew = `LIGHT_GREEN;
                light_ped = `PED_EW; 
                timer_en = 1; 
                timer_load = 0; 

                saver = `EW_YELLOW;
                next_state = `LOADER;
                
                // set next value for internal registers
                // sample_reg_next = 1; 
            end
            `EW_YELLOW: begin
                light_ns = `LIGHT_RED;
                light_ew = `LIGHT_YELLOW;
                light_ped = `PED_EW; 
                timer_en = 1; 
                timer_load = 0; 

                if (ped)
                    saver = `PED_3;
                else
                    saver = `EW_GREEN;
                    
                next_state = `LOADER;
                
                // set next value for internal registers
                // sample_reg_next = 1; 
            end
            `PED_3: begin
                light_ns = `LIGHT_RED;
                light_ew = `LIGHT_RED;
                light_ped = `PED_BOTH; 
                timer_en = 1; 
                timer_load = 0; 

                if (car_ew)
                    if (car_ns)
                        saver = `NS_GREEN;
                    else
                        saver = `EW_GREEN;
                else
                    saver = `NS_GREEN;
                    
                next_state = `LOADER;
                
                // set next value for internal registers
                // sample_reg_next = 1; 
            end
            default: begin
                light_ns = `LIGHT_RED;
                light_ew = `LIGHT_RED;
                light_ped = `PED_NEITHER; 
                timer_en = 0;
                timer_init = 4'b0000; 
                timer_load = 0;

                saver = `IDLE; 
                next_state = `LOADER;
                
                // sample_reg_next = 0; 
            end
        endcase
     end
    

    // The CPU interfaces with main memory which is enabled by the
    // inputs and outputs of this module (r_data, wr_en, mem_addr, w_data)
    // You should create the register file, flip flops, and logic implementing
    // a simple datapath so that instructions can be loaded from main memory,
    // executed, and the results can be inspected in the register file, or in
    // main memory (once lw and sw are supported). You should also create a
    // control FSM that controls the behavior of the datapath depending on the
    // instruction that is currently executing. You may want to split the CPU
    // into one or more submodules.
    //
    // We have provided modules for you to use inside the CPU. Please see
    // the following files:
    // reg_file.sv (register file), reg_en.sv (register with enable and reset),
    // reg_reset.sv (register with only reset), alu.sv (ALU)
    // Useful constants and opcodes are provided in cpu.svh, which is included
    // at the top of this file.
    //
    // Place the instruction machine code (generated by your assembler, or the
    // provided assembler) in asm/instr.mem and it will be automatically
    // loaded into main memory starting at address 0x400000. Make sure the memory
    // file is imported into Vivado first (`./tcl.sh refresh`).
endmodule
