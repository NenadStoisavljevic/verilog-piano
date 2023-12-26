`include "vga.v"
`include "audio.v"
`include "vga_adapter/vga_pll.v"
`include "vga_adapter/vga_controller.v"
`include "vga_adapter/vga_adapter.v"
`include "ps2_keyboard/PS2_Keyboard_Controller.v"
`include "audio/Audio_Controller.v"
`include "memory/recording.v"

// Top module

module piano (
    CLOCK_50,
    KEY, SW, LEDR,
    PS2_CLK, PS2_DAT,
    HEX0, HEX1, HEX2, HEX3, HEX4, HEX5,
    VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_R, VGA_G, VGA_B,
    AUD_ADCDAT, AUD_BCLK, AUD_ADCLRCK, AUD_DACLRCK, FPGA_I2C_SDAT, AUD_XCK, AUD_DACDAT, FPGA_I2C_SCLK
);

    // Inputs and outputs for FPGA
    input CLOCK_50;

    // For audio
    input AUD_ADCDAT;
    inout AUD_BCLK;
    inout AUD_ADCLRCK;
    inout AUD_DACLRCK;
    inout FPGA_I2C_SDAT;
    output AUD_XCK;
    output AUD_DACDAT;
    output FPGA_I2C_SCLK;

    wire co, do, eo, fo, go, ao, bo, cso, dso, fso, gso, aso;
    wire co1, do1, eo1, fo1, go1, ao1, bo1, cso1, dso1, fso1, gso1, aso1;
    wire [1:0] playback;

    // For debugging or auxiliary input
    input [9:0] SW;
    input [3:0] KEY;

    // For PS2 keyboard input and output
    inout PS2_CLK;
    inout PS2_DAT;

    // For debugging or auxiliary output
    output [9:0] LEDR;
    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

    // For VGA DAC signals
    output VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N;
    output [9:0] VGA_R, VGA_G, VGA_B;

    // Wires
    wire clk = CLOCK_50;
    wire reset = !KEY[0];

    wire fill_piano_bg;
    wire fill_c_key, fill_d_key, fill_e_key, fill_f_key, fill_g_key, fill_a_key, fill_b_key;
    wire fill_asbf_key, fill_csdf_key, fill_dsef_key, fill_fsgf_key, fill_gsaf_key;

    wire plot_complete;

    wire plot;
    wire [8:0] x, y;
    wire [2:0] colour;

    // VGA wires
    wire VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N;
    wire [9:0] VGA_R, VGA_G, VGA_B;

    // Keyboard wires
    wire c_en, d_en, e_en, f_en, g_en, a_en, b_en;
    wire csdf_en, dsef_en, fsgf_en, gsaf_en, asbf_en;
    wire key_pressed = c_en || d_en || e_en || f_en || g_en || a_en || b_en ||
                       csdf_en || dsef_en || fsgf_en || gsaf_en || asbf_en;

    // Display "S" when a sharp key is pressed
    assign HEX0 = (csdf_en || dsef_en || fsgf_en || gsaf_en || asbf_en) ? 7'bb001_0010 : 7'h7F;
    
    hex_decoder hd0 (
        .c_en(c_en), .d_en(d_en), .e_en(e_en), .f_en(f_en), .g_en(g_en), .a_en(a_en), .b_en(b_en),
        .csdf_en(csdf_en), .dsef_en(dsef_en), .fsgf_en(fsgf_en), .gsaf_en(gsaf_en), .asbf_en(asbf_en),
        .out(HEX1)
    );

    assign HEX2 = 7'h7F;
    assign HEX3 = 7'h7F;
    assign HEX4 = 7'h7F;
    assign HEX5 = 7'h7F;
    
    // Datapath and control instantiations

    datapath d0 (
        .clk(clk),
        .fill_piano_bg(fill_piano_bg),
        .fill_c_key(fill_c_key), .fill_d_key(fill_d_key), .fill_e_key(fill_e_key),
        .fill_f_key(fill_f_key), .fill_g_key(fill_g_key), .fill_a_key(fill_a_key),
        .fill_b_key(fill_b_key),
        .fill_asbf_key(fill_asbf_key), .fill_csdf_key(fill_csdf_key), .fill_dsef_key(fill_dsef_key),
        .fill_fsgf_key(fill_fsgf_key), .fill_gsaf_key(fill_gsaf_key),
        .plot_complete(plot_complete),
        .plot(plot), .x(x), .y(y), .colour(colour)
    );

    control c0 (
        .clk(clk), .reset(reset), .plot_complete(plot_complete),
        .key_pressed(key_pressed),
        .c_en(c_en), .d_en(d_en), .e_en(e_en), .f_en(f_en), .g_en(g_en), .a_en(a_en), .b_en(b_en),
        .csdf_en(csdf_en), .dsef_en(dsef_en), .fsgf_en(fsgf_en), .gsaf_en(gsaf_en), .asbf_en(asbf_en),
        .fill_piano_bg(fill_piano_bg),
        .fill_c_key(fill_c_key), .fill_d_key(fill_d_key), .fill_e_key(fill_e_key),
        .fill_f_key(fill_f_key), .fill_g_key(fill_g_key), .fill_a_key(fill_a_key),
        .fill_asbf_key(fill_asbf_key), .fill_csdf_key(fill_csdf_key), .fill_dsef_key(fill_dsef_key),
        .fill_fsgf_key(fill_fsgf_key), .fill_gsaf_key(fill_gsaf_key),
        .fill_b_key(fill_b_key)
    );

    // VGA adapter
    vga_adapter #(
        .RESOLUTION("160x120"),
        .MONOCHROME("FALSE"),
        .BITS_PER_COLOUR_CHANNEL(1),
        .BACKGROUND_IMAGE("mifs/white_keys.mif")
    ) vga (
        .clock(clk), .resetn(!reset),

        // Controlled signals
        .x(x), .y(y), .colour(colour),
        .plot(plot),

        // VGA DAC signals
        .VGA_CLK(VGA_CLK),
        .VGA_HS(VGA_HS), .VGA_VS(VGA_VS), .VGA_BLANK(VGA_BLANK_N), .VGA_SYNC(VGA_SYNC_N),
        .VGA_R(VGA_R), .VGA_G(VGA_G), .VGA_B(VGA_B)
    );
    
    // Keyboard tracker
    keyboard_tracker #(.PULSE_OR_HOLD(0)) k0 (
        .clock(CLOCK_50),
        .reset(!reset),
        .PS2_CLK(PS2_CLK),
        .PS2_DAT(PS2_DAT),
        .a(c_en), .s(d_en), .d(e_en), .f(f_en), .g(g_en), .h(a_en), .j(b_en),
        .w(csdf_en), .e(dsef_en), .t(fsgf_en), .y(gsaf_en), .u(asbf_en)
        );
    
    // Audio speakers
    audio a0 (
        .CLOCK_50(CLOCK_50),
        .KEY(!reset),
        .AUD_ADCDAT(AUD_ADCDAT),
        .AUD_BCLK(AUD_BCLK),
        .AUD_ADCLRCK(AUD_ADCLRCK),
        .AUD_DACLRCK(AUD_DACLRCK),
        .FPGA_I2C_SDAT(FPGA_I2C_SDAT),
        .AUD_XCK(AUD_XCK),
        .AUD_DACDAT(AUD_DACDAT),
        .FPGA_I2C_SCLK(FPGA_I2C_SCLK),
        .cIn(c_en || co || co1), .dIn(d_en || do || do1), .eIn(e_en || eo || eo1),
        .fIn(f_en || fo || fo1), .gIn(g_en || go || go1), .aIn(a_en || ao || ao1),
        .bIn(b_en || bo || bo1),
        .csIn(csdf_en || cso || cso1),
        .dsIn(dsef_en || dso || dso1),
        .fsIn(fsgf_en || fso || fso1),
        .gsIn(gsaf_en || gso || gso1),
        .asIn(asbf_en || aso || aso1)
    );

    // Memory playback
    recording r0 (
        .CLOCK_50(CLOCK_50), .active(SW[0]),
        .KEY(KEY),
        .LEDR({playback[0], LEDR[0]}),
        .ci(c_en), .di(d_en), .ei(e_en), .fi(f_en), .gi(g_en), .ai(a_en), .bi(b_en),
        .csi(csdf_en), .dsi(dsef_en), .fsi(fsgf_en), .gsi(gsaf_en), .asi(asbf_en),
        .co(co), .do(do), .eo(eo), .fo(fo), .go(go), .ao(ao), .bo(bo), .cso(cso),
        .dso(dso), .fso(fso), .gso(gso), .aso(aso)
    );
    
    recording r1 (
        .CLOCK_50(CLOCK_50), .active(SW[1]),
        .KEY(KEY),
        .LEDR({playback[1], LEDR[1]}),
        .ci(c_en), .di(d_en), .ei(e_en), .fi(f_en), .gi(g_en), .ai(a_en), .bi(b_en),
        .csi(csdf_en), .dsi(dsef_en), .fsi(fsgf_en), .gsi(gsaf_en), .asi(asbf_en),
        .co(co1), .do(do1), .eo(eo1), .fo(fo1), .go(go1), .ao(ao1), .bo(bo1), .cso(cso1),
        .dso(dso1), .fso(fso1), .gso(gso1), .aso(aso1)
    );

    assign LEDR[9] = |playback;

    // Counter

    integer counter;
    integer count = 25000000;
    reg metronome;

    always @(posedge CLOCK_50) begin
        if (counter == count) begin
            counter <= 0;
            metronome <= !metronome;
        end
        else counter <= counter + 1;
    end

    assign LEDR[8] = metronome;

endmodule // Top module

// Datapath

module datapath (
    clk,
    fill_piano_bg,
    fill_c_key, fill_d_key, fill_e_key, fill_f_key, fill_g_key, fill_a_key, fill_b_key,
    fill_asbf_key, fill_csdf_key, fill_dsef_key, fill_fsgf_key, fill_gsaf_key,
    plot_complete,
    plot, x, y, colour
);

    // Inputs, outputs and wires

    input clk;

    input fill_piano_bg;
    input fill_c_key, fill_d_key, fill_e_key, fill_f_key, fill_g_key, fill_a_key, fill_b_key;
    input fill_asbf_key, fill_csdf_key, fill_dsef_key, fill_fsgf_key, fill_gsaf_key;

    output plot_complete;

    wire plot_complete_bg, plot_complete_piano_key;
    assign plot_complete = plot_complete_bg || plot_complete_piano_key;

    wire [8:0] next_x_piano_bg, next_x_piano_key;
    wire [8:0] next_y_piano_bg, next_y_piano_key;
    output reg [2:0] colour;

    output plot;
    output reg [8:0] x;
    output reg [8:0] y;
    
    reg sync_plot;
    assign plot = sync_plot && !plot_complete;

    wire draw, fill_piano_key;
    assign draw = fill_piano_bg || fill_piano_key;
    assign fill_piano_key = fill_c_key || fill_d_key || fill_e_key ||
           fill_f_key || fill_g_key || fill_a_key || fill_b_key ||
           fill_asbf_key || fill_csdf_key || fill_dsef_key ||
           fill_fsgf_key || fill_gsaf_key;

    always @(posedge clk) begin
        sync_plot <= draw;

        if (fill_c_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else if (fill_d_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else if (fill_e_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else if (fill_f_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else if (fill_g_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else if (fill_a_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else if (fill_b_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else if (fill_asbf_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else if (fill_csdf_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else if (fill_dsef_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else if (fill_fsgf_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else if (fill_gsaf_key) begin
            x <= next_x_piano_key;
            y <= next_y_piano_key;
        end else begin
            x <= next_x_piano_bg;
            y <= next_y_piano_bg;
        end 
    end

    // Plotters for MIF files

    plotter #(
        .WIDTH_X(9),
        .WIDTH_Y(9),
        .MAX_X(160),
        .MAX_Y(120)
    ) plt_piano_bg (
        .clk(clk), .en(fill_piano_bg && !plot_complete),
        .x(next_x_piano_bg), .y(next_y_piano_bg),
        .done(plot_complete_bg)
    );

    plotter #(
        .WIDTH_X(9),
        .WIDTH_Y(9),
        .MAX_X(160),
        .MAX_Y(120)
    ) plt_piano_key (
        .clk(clk), .en(fill_piano_key && !plot_complete),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .done(plot_complete_piano_key)
    );

    // Piano background screen

    wire [2:0] piano_bg_colour;

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/white_keys.mif")
    ) srm_piano_bg (
        .clk(clk),
        .x(next_x_piano_bg), .y(next_y_piano_bg),
        .colour_out(piano_bg_colour)
    );

    // RAM modules for MIF files

    wire [2:0] c_key_colour, d_key_colour, e_key_colour, f_key_colour, g_key_colour, a_key_colour, b_key_colour;
    wire [2:0] asbf_key_colour, csdf_key_colour, dsef_key_colour, fsgf_key_colour, gsaf_key_colour;

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/c_key.mif")
    ) c_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(c_key_colour)
    );

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/d_key.mif")
    ) d_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(d_key_colour)
    );

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/e_key.mif")
    ) e_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(e_key_colour)
    );

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/f_key.mif")
    ) f_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(f_key_colour)
    );

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/g_key.mif")
    ) g_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(g_key_colour)
    );

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/a_key.mif")
    ) a_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(a_key_colour)
    );

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/b_key.mif")
    ) b_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(b_key_colour)
    );
    
    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/asbf_key.mif")
    ) asbf_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(asbf_key_colour)
    );

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/csdf_key.mif")
    ) csdf_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(csdf_key_colour)
    );

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/dsef_key.mif")
    ) dsef_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(dsef_key_colour)
    );

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/fsgf_key.mif")
    ) fsgf_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(fsgf_key_colour)
    );

    sprite_ram_module #(
        .WIDTH_X(8),
        .WIDTH_Y(7),
        .RESOLUTION_X(160),
        .RESOLUTION_Y(120),
        .MIF_FILE("mifs/gsaf_key.mif")
    ) gsaf_key (
        .clk(clk),
        .x(next_x_piano_key), .y(next_y_piano_key),
        .colour_out(gsaf_key_colour)
    );

    // Colour selector

    always @ (*) begin
        if (fill_c_key)
            colour = c_key_colour;
        else if (fill_d_key)
            colour = d_key_colour;
        else if (fill_e_key)
            colour = e_key_colour;
        else if (fill_f_key)
            colour = f_key_colour;
        else if (fill_g_key)
            colour = g_key_colour;
        else if (fill_a_key)
            colour = a_key_colour;
        else if (fill_b_key)
            colour = b_key_colour;
        else if (fill_asbf_key)
            colour = asbf_key_colour;
        else if (fill_csdf_key)
            colour = csdf_key_colour;
        else if (fill_dsef_key)
            colour = dsef_key_colour;
        else if (fill_fsgf_key)
            colour = fsgf_key_colour;
        else if (fill_gsaf_key)
            colour = gsaf_key_colour;
        else
            colour = piano_bg_colour;
    end

endmodule // Datapath

// Control

module control (
    clk, reset, plot_complete, key_pressed,
	c_en, d_en, e_en, f_en, g_en, a_en, b_en,
    csdf_en, dsef_en, fsgf_en, gsaf_en, asbf_en,
    fill_piano_bg,
	fill_c_key, fill_d_key, fill_e_key, fill_f_key, fill_g_key, fill_a_key, fill_b_key,
    fill_asbf_key, fill_csdf_key, fill_dsef_key, fill_fsgf_key, fill_gsaf_key,
);

    input clk, reset, plot_complete, key_pressed;
	input c_en, d_en, e_en, f_en, g_en, a_en, b_en;
    input csdf_en, dsef_en, fsgf_en, gsaf_en, asbf_en;

    output reg fill_piano_bg;
	output reg fill_c_key, fill_d_key, fill_e_key, fill_f_key, fill_g_key, fill_a_key, fill_b_key;
    output reg fill_asbf_key, fill_csdf_key, fill_dsef_key, fill_fsgf_key, fill_gsaf_key;

    reg [4:0] current_state;
    reg [4:0] next_state;

    // States
    localparam  S_FILL_PIANO_BG  = 0,    // Fill piano background
                S_WAIT_PIANO_KEY = 1,    // Wait before drawing piano keys
	            S_FILL_PIANO_KEY = 2,    // Fill piano keys
                S_FILL_C_KEY     = 3,    // Fill C key
                S_FILL_D_KEY     = 4,    // Fill D key
                S_FILL_E_KEY     = 5,    // Fill E Key
                S_FILL_F_KEY     = 6,    // Fill F key
                S_FILL_G_KEY     = 7,    // Fill G key
                S_FILL_A_KEY     = 8,    // Fill A key
                S_FILL_B_KEY     = 9,    // Fill B key
                S_FILL_ASBF_KEY  = 10,   // Fill ASBF Key
                S_FILL_CSDF_KEY  = 11,   // Fill CSDF key
                S_FILL_DSEF_KEY  = 12,   // Fill DSEF key
                S_FILL_FSGF_KEY  = 13,   // Fill FSGF key
                S_FIll_GSAF_KEY  = 14;   // Fill GSAF key

    // State table
    always @ (*) begin
        case (current_state)
            S_FILL_PIANO_BG:
            next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_PIANO_BG;
			S_WAIT_PIANO_KEY:
            next_state = key_pressed ? S_FILL_PIANO_KEY : S_FILL_PIANO_BG;
			S_FILL_PIANO_KEY: begin
                if (c_en) next_state         = S_FILL_C_KEY;
				else if (d_en) next_state    = S_FILL_D_KEY;
				else if (e_en) next_state    = S_FILL_E_KEY;
				else if (f_en) next_state    = S_FILL_F_KEY;
				else if (g_en) next_state    = S_FILL_G_KEY;
				else if (a_en) next_state    = S_FILL_A_KEY;
				else if (b_en) next_state    = S_FILL_B_KEY;
                else if (asbf_en) next_state = S_FILL_ASBF_KEY;
				else if (csdf_en) next_state = S_FILL_CSDF_KEY;
				else if (dsef_en) next_state = S_FILL_DSEF_KEY;
				else if (fsgf_en) next_state = S_FILL_FSGF_KEY;
				else if (gsaf_en) next_state = S_FIll_GSAF_KEY;
                else next_state              = S_WAIT_PIANO_KEY;
            end
            S_FILL_C_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_C_KEY;
            S_FILL_D_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_D_KEY;
            S_FILL_E_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_E_KEY;
            S_FILL_F_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_F_KEY;
            S_FILL_G_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_G_KEY;
            S_FILL_A_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_A_KEY;
            S_FILL_B_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_B_KEY;
            S_FILL_ASBF_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_ASBF_KEY;
            S_FILL_CSDF_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_CSDF_KEY;
            S_FILL_DSEF_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_DSEF_KEY;
            S_FILL_FSGF_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FILL_FSGF_KEY;
            S_FIll_GSAF_KEY: next_state = plot_complete ? S_WAIT_PIANO_KEY : S_FIll_GSAF_KEY;
			default: next_state = S_FILL_PIANO_BG;
        endcase
    end

    // State switching and reset
    always @ (posedge clk) begin
        if (reset)
            current_state <= S_FILL_PIANO_BG;
        else
            current_state <= next_state;
    end

    // Output logic
    always @ (*) begin
        // Reset control signals
        fill_piano_bg = 0;
		fill_c_key         = 0;
		fill_d_key         = 0;
		fill_e_key         = 0;
		fill_f_key         = 0;
		fill_g_key         = 0;
		fill_a_key         = 0;
		fill_b_key         = 0;
        fill_asbf_key      = 0;
		fill_csdf_key      = 0;
		fill_dsef_key      = 0;
		fill_fsgf_key      = 0;
		fill_gsaf_key      = 0;

        // Set control signals based on state
        case (current_state)
                S_FILL_PIANO_BG: begin
                    fill_piano_bg = 1;
                end
				S_FILL_C_KEY: begin
					fill_c_key = 1;
				end
				S_FILL_D_KEY: begin
					fill_d_key = 1;
				end
				S_FILL_E_KEY: begin
					fill_e_key = 1;
				end
				S_FILL_F_KEY: begin
					fill_f_key = 1;
				end
				S_FILL_G_KEY: begin
					fill_g_key = 1;
				end
				S_FILL_A_KEY: begin
					fill_a_key = 1;
				end
				S_FILL_B_KEY: begin
					fill_b_key = 1;
				end
                S_FILL_ASBF_KEY: begin
					fill_asbf_key = 1;
				end
				S_FILL_CSDF_KEY: begin
					fill_csdf_key = 1;
				end
				S_FILL_DSEF_KEY: begin
					fill_dsef_key = 1;
				end
				S_FILL_FSGF_KEY: begin
					fill_fsgf_key = 1;
				end
				S_FIll_GSAF_KEY: begin
					fill_gsaf_key = 1;
				end

        endcase
    end

endmodule // Control

// Hex display

module hex_decoder(
    c_en, d_en, e_en, f_en, g_en, a_en, b_en,
    csdf_en, dsef_en, fsgf_en, gsaf_en, asbf_en,
    out
);
    input c_en, d_en, e_en, f_en, g_en, a_en, b_en;
    input csdf_en, dsef_en, fsgf_en, gsaf_en, asbf_en;
    output reg [6:0] out;

    always @(*) begin
        if (c_en || csdf_en) out = 7'b100_0110;
        else if (d_en || dsef_en) out = 7'b010_0001;
        else if (e_en) out = 7'b000_0110;
        else if (f_en || fsgf_en) out = 7'b000_1110;
        else if (g_en || gsaf_en) out = 7'b000_0010;
        else if (a_en || asbf_en) out = 7'b000_1000;
        else if (b_en) out = 7'b000_0011;
        else out = 7'h7f;
    end
endmodule // Hex display