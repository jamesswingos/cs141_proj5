// For this project the SystemVerilog testbench is
// provided, you do not need to modify it.
//
// You will have to write one or multiple MIPS programs
// that test the functionality of your CPU, and assemble
// them using your assembler from project 4, or the provided
// assembler. Then you can run this testbench and observe
// the results in the waveform viewer.
//
// Please place the machine code output of the assembler in
// the `asm/instr.mem` file, and make sure to import that
// file into your vivado project (`./tcl.sh refresh`). The
// RAM module that we provide will automatically load it
// into the memory space starting at 0x400000. Any hex data
// in asm/data.mem will also be automatically loaded into RAM
// starting at address 0.
module cpu_tb();
    logic clk_100M, clk, rst;

    logic wr_en;
    logic [31:0] mem_addr, w_data, r_data;
    logic [31:0] dbg_addr, mem_rdbg_data;
    logic [31:0] instr;
    logic [31:0] rdbg_addr;
    logic [31:0] rdbg_data;
    
    rw_ram ram_unit (.*, .addr(mem_addr), .rdbg_data(mem_rdbg_data));
    cpu cpu_unit (.*);

    initial begin
        rst <= 1;
        # 22;
        rst <= 0;
    end

    always begin
        // clock does not need to be
        // divided for simulation
        clk <= 1;
        clk_100M <= 1;
        #5;
        clk <= 0;
        clk_100M <= 0;
        #5;
    end
endmodule
