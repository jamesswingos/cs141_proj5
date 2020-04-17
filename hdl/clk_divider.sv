module clk_divider
    #(parameter N = 28)
    (
        input logic clk, rst,
        input logic [N-1:0] max,
        output logic out_clk
    );

    logic [N-1:0] r_reg;
    logic [N-1:0] r_next;
    
    always_ff @(posedge clk, posedge rst)
        if (rst)
            r_reg <= 0;
        else
            r_reg <= r_next;
    
    always_ff @(posedge clk, posedge rst)
        if (rst)
            out_clk <= 0;
        else if (r_reg >= max)
            out_clk <= ~out_clk;
    
    assign r_next = (r_reg >= max) ? 0 : r_reg + 1;
endmodule
