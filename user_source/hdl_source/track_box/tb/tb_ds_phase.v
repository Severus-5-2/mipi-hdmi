//=====================================================================
//  tb_ds_phase.v  ——  v5.2 downsample VERTICAL PHASE probe  (zmc 2026-10-19)
//---------------------------------------------------------------------
//  WHY THIS TB IS NEEDED
//    tb_motion_detect.v uses large blocks (128..447 etc.), so row 4r+1 and
//    row 4r+3 inside a block are IDENTICAL.  Therefore that 12-frame golden
//    vector CANNOT tell <y[1:0]==3> from <y[1:0]==1>.
//    Measured: switching ==3 to ==1 leaves all 12 golden frames unchanged.
//    => if someone later writes ==1, the existing TB stays green, but the
//       whole ds map shifts UP by 2 lines and every box is offset.
//
//  HOW IT DISCRIMINATES
//    Make the 4 rows of each group have DIFFERENT grey levels:
//        y%4 = 0 -> 10    1 -> 40    2 -> 90    3 -> 160
//    All x within a row are equal (so rgb2gray's 1-cycle latency is harmless).
//    Average of 2 equal pixels = that level, so the printed ds IS the grey
//    level of the sampled row.
//
//  READING THE OUTPUT
//      ds = 160  -> samples 4r+3   OK   (v5.2 / zero-storage baseline)
//      ds =  40  -> samples 4r+1   ERR  (phase off by 2 lines)
//      ds =  90  -> samples 4r+2   ERR
//      ds =  10  -> samples 4r+0   ERR
//
//  RUN (from track_box/)
//      iverilog -g2005 -o /tmp/phase.vvp tb/tb_ds_phase.v rtl/motion_detect.v ../rgb2gray.v
//      vvp /tmp/phase.vvp
//    Expect 12 lines "PHASE r=.. c=.. ds=160 OK" then "PHASE_RESULT PASS".
//    Column c=0 is the left edge (window touches x=-1); it is reported but
//    NOT counted in the pass/fail verdict.
//=====================================================================
`timescale 1ns / 1ps

module tb_ds_phase;

    reg         clk = 0;
    reg         rst_n = 0;
    reg         de = 0;
    reg  [10:0] px = 0;
    reg  [9:0]  py = 0;
    reg  [23:0] rgb = 0;
    reg         fdone = 0;

    wire [3:0]  o_valid;
    wire [43:0] o_xmin, o_xmax;
    wire [39:0] o_ymin, o_ymax;
    wire [2:0]  o_cnt;
    wire        o_pix, o_pix_v;
    wire [7:0]  o_act;

    motion_detect dut (
        .I_clk        (clk),
        .I_rst_n      (rst_n),
        .I_de         (de),
        .I_pix_x      (px),
        .I_pix_y      (py),
        .I_rgb        (rgb),
        .I_frame_done (fdone),
        .O_mot_valid  (o_valid),
        .O_mot_xmin   (o_xmin),
        .O_mot_xmax   (o_xmax),
        .O_mot_ymin   (o_ymin),
        .O_mot_ymax   (o_ymax),
        .O_mot_cnt    (o_cnt),
        .O_mot_pix    (o_pix),
        .O_mot_pix_v  (o_pix_v),
        .O_act_blk    (o_act)
    );

    always #5 clk = ~clk;

    // per-row grey levels inside one 4x4 group
    function [7:0] rowval(input [1:0] ph);
        case (ph)
            2'd0:    rowval = 8'd10;
            2'd1:    rowval = 8'd40;
            2'd2:    rowval = 8'd90;
            default: rowval = 8'd160;
        endcase
    endfunction

    integer r, c, k, n_bad, n_got;
    reg [7:0] got [0:3][0:3];

    initial begin
        n_bad = 0;
        n_got = 0;
        for (r = 0; r < 4; r = r + 1)
            for (c = 0; c < 4; c = c + 1)
                got[r][c] = 8'hxx;

        repeat (4) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // 4 ds rows (y = 0..15) x 32 px (2 block columns), 1 pixel per clock
        for (k = 0; k < 16; k = k + 1) begin
            for (c = 0; c < 32; c = c + 1) begin
                de  = 1'b1;
                px  = c[10:0];
                py  = k[9:0];
                // R = G = B = v  =>  rgb2gray outputs exactly v
                rgb = {rowval(k[1:0]), rowval(k[1:0]), rowval(k[1:0])};
                @(posedge clk);
            end
        end
        de = 1'b0;
        repeat (8) @(posedge clk);

        for (r = 0; r < 4; r = r + 1) begin
            for (c = 0; c < 4; c = c + 1) begin
                if (got[r][c] === 8'hxx) begin
                    $display("PHASE r=%0d c=%0d MISSING", r, c);
                    if (c != 0) n_bad = n_bad + 1;
                end else begin
                    $display("PHASE r=%0d c=%0d ds=%0d %s", r, c, got[r][c],
                             (got[r][c] == 8'd160) ? "OK  (samples 4r+3)" :
                             (got[r][c] == 8'd40 ) ? "ERR (samples 4r+1, phase -2 lines)" :
                             (got[r][c] == 8'd90 ) ? "ERR (samples 4r+2)" :
                             (got[r][c] == 8'd10 ) ? "ERR (samples 4r+0)" : "ERR (other)");
                    if ((c != 0) && (got[r][c] !== 8'd160)) n_bad = n_bad + 1;
                end
                if (c != 0) n_got = n_got + 1;
            end
        end

        $display("----------------------------------------------------------");
        if (n_bad == 0)
            $display("PHASE_RESULT PASS  sampled row = 4r+3 (vertical phase OK)");
        else
            $display("PHASE_RESULT FAIL  %0d/%0d points wrong (see table above)", n_bad, n_got);
        $display("----------------------------------------------------------");
        $finish;
    end

    // capture valid ds samples: dsx = I_pix_x[10:2], dsy = I_pix_y[9:2]
    always @(posedge clk) begin
        if (rst_n && dut.S_ds_v && (dut.S_dsy < 8'd4) && (dut.S_dsx < 9'd4))
            got[dut.S_dsy[1:0]][dut.S_dsx[1:0]] = dut.S_ds;
    end

endmodule
