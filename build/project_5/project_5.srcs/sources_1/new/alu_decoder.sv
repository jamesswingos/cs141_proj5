//`include "cpu.svh"

module alu_decoder(
    input logic [1:0] ALUOp,
    input logic [5:0] funct,
    output logic [3:0] ALUControl
    );
    
// ALUControl changes based on ALUOp and function code
always_comb begin
    case(ALUOp)
        2'b00: ALUControl = `ALU_ADD;
        2'b01: ALUControl = `ALU_SUB;
        2'b10: begin
            case(funct)
                `F_AND: ALUControl = `ALU_AND;
                `F_OR: ALUControl = `ALU_OR;
                `F_XOR: ALUControl = `ALU_XOR;
                `F_NOR: ALUControl = `ALU_NOR;
                `F_SLL: ALUControl = `ALU_SLL;
                `F_SRL: ALUControl = `ALU_SRL;
                `F_SRA: ALUControl = `ALU_SRA;
                `F_SLT: ALUControl = `ALU_SLT;
                `F_ADD: ALUControl = `ALU_ADD;
                `F_SUB: ALUControl = `ALU_SUB;
            endcase
         end
    endcase
end
endmodule
