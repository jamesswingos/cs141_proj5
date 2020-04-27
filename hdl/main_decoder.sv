module main_decoder(
    input logic clk, rst,
    input [5:0] opcode,
    input [5:0] funct,
    
    // enabling registers
    output logic IRWrite, RegWrite, MemWrite,
    
    // pc enabling
    output logic Branch, PCWrite,
    
    // multiplexing
    output logic IorD, RegDst, MemtoReg, ALUSrcA, PCSrc,
    output logic [1:0] ALUSrcB,
    output logic [1:0] ALUOp
    );
    
    logic [5:0] state, next_state;
     always_ff @(posedge clk, posedge rst) begin
        if (rst) begin
            state <= `S0;
        end
        else begin
            state <= next_state;   
        end
     end
    
    always_comb begin
        next_state = state;
        unique case (state)
            `S0: begin
                // set new controls
                IorD = 0;
                RegDst = 1'b1;
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
                case (opcode)
                    `OP_RTYPE: next_state = `S6;
                    `OP_LW: next_state = `S2;
                    `OP_SW: next_state = `S2;
                    `OP_BEQ: next_state = `S8;
                    `OP_ADDI: next_state = `S9;
                    `OP_J: next_state = `S11;
                 endcase
           end                
           `S2: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b10;
                ALUOp = 2'b11;
                if (opcode == `OP_LW) begin
                    next_state = `S3;
                end
                else
                    if (opcode == `OP_SW) begin
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
                MemWrite = 1'b1;
                next_state = `S0;
           end
           `S6: begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b00;
                ALUOp = 2'b10;
                next_state = `S7;
                RegDst = 1'b1;
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
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b10;
                ALUOp = 2'b00;
                next_state = `S10;
           end
           `S10: begin
                RegDst = 1'b0;
                MemtoReg = 1'b0;
                RegWrite = 1'b1;
                next_state = `S0;
           end
           `S11: begin
                PCSrc = 2'b10;
                PCWrite = 1'b1;
                next_state = `S0;
           end
                
            default: begin
                IorD = 0;
                MemWrite = 0;
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b01; 
                ALUOp = 2'b00; 
                PCSrc = 2'b00;
                IRWrite = 1'b1;
                PCWrite = 1'b1;
                next_state = `S1;
            end
        endcase
     end
    
endmodule
