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
    
    // first: wire up the datapath w/o controls
    // control variables for the datapath
    logic IorD, IRWrite, RegDst, MemtoReg, RegWrite, ALUSrcA, PCSrc, Branch, PCWrite;
    logic [1:0] ALUSrcB;
    logic [1:0] ALUOp;
    
    // output from ALU decoder
    logic [3:0] ALUControl;
    
    // intermediary values to save
        // main datapath
        logic [31:0] minst, temp2, reg_Asrc, reg_Bsrc, wd3, SrcB;
        logic [31:0] alu_out;
        // for sign extension
        logic [31:0] r_ext;
        logic [31:0] r_sft;    
        // ALU results
        logic [31:0] alu_res;
        logic zero;    
        // for branching logic
        logic b_temp, PC_en;
        // saved values from instruction
        logic [4:0] rf_a0, rf_a1, adbg, dest;
        logic [5:0] funct;
        logic [5:0] opcode;
         // outputs of register file
        logic [31:0] rf_r0, rf_r1, rdbg;
        
        // source A temps
        logic [31:0] pc_stage;
        logic [31:0] SrcA;

    // run the controller
    main_decoder main_decoder_unit (.*, .MemWrite(wr_en)); // assign values to muxers and enablers
    alu_decoder alu_decoder_unit (.*); // get the ALUControl signal
    
    // final register for the PC enabling
    reg_en #(.INIT(32'h400000)) reg_6 (.*, .en(PC_en), .d(pc_nxt), .q(pc_stage));
    
    // multiplex on PC choice
    logic [31:0] mem_fin;
    assign mem_addr = IorD ? alu_out : pc_stage;
    
    // save vals for first two reg's
    reg_en reg_1 (.*, .en(IRWrite), .d(r_data), .q(minst));  
    reg_reset reg_2 (.*, .d(r_data), .q(temp2));
    
    // choose which reg to save result to (r vs. i/j)
    assign dest = RegDst ? minst[15:11] : minst[20:16];
    
    // saving the instruction values
    assign opcode = minst[31:26];
    assign funct = minst[5:0];
    assign rf_a0 = minst[25:21];
    assign rf_a1 = minst[20:16];
        
    // choose b/w ALUOut and memory output (r vs. i/j)
    assign wd3 = MemtoReg ? temp2 : alu_out;
    
    // interact w/ register file
    reg_file reg_main (.clk, .wr_en(RegWrite), .w_addr(dest), .r0_addr(rf_a0),
                       .r1_addr(rf_a1), .w_data(wd3), .r0_data(rf_r0),
                       .r1_data(rf_r1), .rdbg_addr(adbg), .rdbg_data(rdbg));
    
    // build results from FFs following register file
    reg_reset reg_A (.clk, .rst, .d(rf_r0), .q(reg_Asrc));
    reg_reset reg_B (.clk, .rst, .d(rf_r1), .q(reg_Bsrc));
    
    // multiplex for ALU SrcA
    assign SrcA = ALUSrcA ? reg_Asrc : pc_stage;
    
    // sign extend
    assign r_ext = minst[15] ? {16'b1111111111111111, minst[15:0]} : {16'b0000000000000000, minst[15:0]};
    
    // shift extension 2 spots
    assign r_sft = r_ext << 2;
    
    // SrcB multiplexer and save w_data
    assign w_data = reg_Bsrc;
    assign SrcB = ALUSrcB[1] ? (ALUSrcB[0] ? r_sft : r_ext) : (ALUSrcB[0] ? 32'b100 : reg_Bsrc);

    // interact with the ALU (adjust op codes for larger inst set)
    alu alu_main (.x(SrcA), .y(SrcB), .op(ALUControl), .z(alu_res), .zero);
    
    // control zero and branching
    assign b_temp = zero && Branch;
    assign PC_en = b_temp || PCWrite;
    
    // another reg for ALU output
    reg_reset reg_5 (.*, .d(alu_res), .q(alu_out));
    
    // multiplex on ALU result
    logic [31:0] pc_nxt;
    assign pc_nxt = PCSrc ? alu_out : alu_res;
    
    
 
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
