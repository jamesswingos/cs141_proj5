module cpu_top
    (
        input logic clk_100M, // 100M clock signal
        input logic [15:0] sw,
        input logic reset_n,
        output logic [0:0] led,
        output logic [7:0] an,
        output logic [7:0] sseg
    );

    logic clk, rst;
    assign rst = ~reset_n;
    assign led[0] = clk;

    logic [27:0] divide_sw;
    assign divide_sw = sw[5:0] << 20;
    clk_divider clk_div_unit (.clk(clk_100M), .reset(1'b0),
        .max(divide_sw), .out_clk(clk));

    logic wr_en;
    logic [31:0] mem_addr, w_data, r_data;
    logic [31:0] dbg_addr, mem_rdbg_data;
    assign dbg_addr = sw[13:6];
    rw_ram ram_unit (.*, .addr(mem_addr), .rdbg_data(mem_rdbg_data));

    logic [31:0] instr;
    logic [4:0] rdbg_addr;
    logic [31:0] rdbg_data;
    
    assign rdbg_addr = sw[13:6];
    cpu cpu_unit (.*);

    logic [31:0] disp;
    assign disp = sw[14] ? instr : sw[15] ? mem_rdbg_data : rdbg_data;

    disp_hex_mux disp_unit (.*, .clk(clk_100M), .hex7(disp[31:28]), .hex6(disp[27:24]),
        .hex5(disp[23:20]), .hex4(disp[19:16]), .hex3(disp[15:12]),
        .hex2(disp[11:8]), .hex1(disp[7:4]), .hex0(disp[3:0]),
        .dp_in(8'b11111111), .reset(1'b0));
endmodule
