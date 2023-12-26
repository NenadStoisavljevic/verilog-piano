`include "avconf/avconf.v"

module audio (
	// Inputs
	CLOCK_50,
	KEY,

	AUD_ADCDAT,

	// Bidirectionals
	AUD_BCLK,
	AUD_ADCLRCK,
	AUD_DACLRCK,

	FPGA_I2C_SDAT,

	// Outputs
	AUD_XCK,
	AUD_DACDAT,

	FPGA_I2C_SCLK,
	cIn, dIn, eIn, fIn, gIn, aIn, bIn,
	csIn, dsIn, fsIn, gsIn, asIn
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				CLOCK_50;
input		[3:0]	KEY;

input				AUD_ADCDAT;

// Bidirectionals
inout				AUD_BCLK;
inout				AUD_ADCLRCK;
inout				AUD_DACLRCK;

inout				FPGA_I2C_SDAT;

// Outputs
output				AUD_XCK;
output				AUD_DACDAT;

output				FPGA_I2C_SCLK;

//notes
input cIn, dIn, eIn, fIn, gIn, aIn, bIn;
input csIn, dsIn, fsIn, gsIn, asIn;

/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;

// Internal Registers

reg [18:0] cDelay, dDelay, eDelay, fDelay, gDelay, aDelay, bDelay;
wire [18:0] cFreq, dFreq, eFreq, fFreq, gFreq, aFreq, bFreq;

reg[18:0] csDelay, dsDelay, fsDelay, gsDelay, asDelay;
wire [18:0] csFreq, dsFreq, fsFreq, gsFreq, asFreq;

reg cSnd, dSnd, eSnd, fSnd, gSnd, aSnd, bSnd;
reg csSnd, dsSnd, fsSnd, gsSnd, asSnd;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/

always @(posedge CLOCK_50)
	if(cDelay == cFreq) begin
		cDelay <= 0;
		cSnd <= !cSnd;
	end else cDelay <= cDelay + 1;
	
always @(posedge CLOCK_50)
	if(dDelay == dFreq) begin
		dDelay <= 0;
		dSnd <= !dSnd;
	end else dDelay <= dDelay + 1;
	
always @(posedge CLOCK_50)
	if(eDelay == eFreq) begin
		eDelay <= 0;
		eSnd <= !eSnd;
	end else eDelay <= eDelay + 1;
	
always @(posedge CLOCK_50)
	if(fDelay == fFreq) begin
		fDelay <= 0;
		fSnd <= !fSnd;
	end else fDelay <= fDelay + 1;
	
always @(posedge CLOCK_50)
	if(gDelay == gFreq) begin
		gDelay <= 0;
		gSnd <= !gSnd;
	end else gDelay <= gDelay + 1;
	
always @(posedge CLOCK_50)
	if(aDelay == aFreq) begin
		aDelay <= 0;
		aSnd <= !aSnd;
	end else aDelay <= aDelay + 1;
	
always @(posedge CLOCK_50)
	if(bDelay == bFreq) begin
		bDelay <= 0;
		bSnd <= !bSnd;
	end else bDelay <= bDelay + 1;
	
//////////////////////////
	
always @(posedge CLOCK_50)
	if(csDelay == csFreq) begin
		csDelay <= 0;
		csSnd <= !csSnd;
	end else csDelay <= csDelay + 1;
	
always @(posedge CLOCK_50)
	if(dsDelay == dsFreq) begin
		dsDelay <= 0;
		dsSnd <= !dsSnd;
	end else dsDelay <= dsDelay + 1;
	
always @(posedge CLOCK_50)
	if(fsDelay == fsFreq) begin
		fsDelay <= 0;
		fsSnd <= !fsSnd;
	end else fsDelay <= fsDelay + 1;
	
always @(posedge CLOCK_50)
	if(gsDelay == gsFreq) begin
		gsDelay <= 0;
		gsSnd <= !gsSnd;
	end else gsDelay <= gsDelay + 1;
	
always @(posedge CLOCK_50)
	if(asDelay == asFreq) begin
		asDelay <= 0;
		asSnd <= !asSnd;
	end else asDelay <= asDelay + 1;
	
/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/

assign cFreq = 18'd47780;
assign dFreq = 18'd42567;
assign eFreq = 18'd37923;
assign fFreq = 18'd35795;
assign gFreq = 18'd31888;
assign aFreq = 18'd28410;
assign bFreq = 18'd25310;

assign csFreq = 18'd45098;
assign dsFreq = 18'd40178;
assign fsFreq = 18'd33784;
assign gsFreq = 18'd30099;
assign asFreq = 18'd26815;

wire [31:0] sound = ((cIn == 0) ? 0 : cSnd ? 32'd100000000 : -32'd100000000)+((dIn == 0) ? 0 : dSnd ? 32'd100000000 : -32'd100000000)+
	((eIn == 0) ? 0 : eSnd ? 32'd100000000 : -32'd100000000)+((fIn == 0) ? 0 : fSnd ? 32'd100000000 : -32'd100000000)+
	((gIn == 0) ? 0 : gSnd ? 32'd100000000 : -32'd100000000)+((aIn == 0) ? 0 : aSnd ? 32'd100000000 : -32'd100000000)+
	((bIn == 0) ? 0 : bSnd ? 32'd100000000 : -32'd100000000)+((csIn == 0) ? 0 : csSnd ? 32'd100000000 : -32'd100000000)+
	((dsIn == 0) ? 0 : dsSnd ? 32'd100000000 : -32'd100000000)+((fsIn == 0) ? 0 : fsSnd ? 32'd100000000 : -32'd100000000)+
	((gsIn == 0) ? 0 : gsSnd ? 32'd100000000 : -32'd100000000)+((asIn == 0) ? 0 : asSnd ? 32'd100000000 : -32'd100000000);

assign read_audio_in			= audio_in_available & audio_out_allowed;

assign left_channel_audio_out	= left_channel_audio_in+sound;
assign right_channel_audio_out	= right_channel_audio_in+sound;
assign write_audio_out			= audio_in_available & audio_out_allowed;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Audio_Controller Audio_Controller (
	// Inputs
	.CLOCK_50						(CLOCK_50),
	.reset						(~KEY[0]),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),


	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)

);

avconf #(.USE_MIC_INPUT(1)) avc (
	.FPGA_I2C_SCLK					(FPGA_I2C_SCLK),
	.FPGA_I2C_SDAT					(FPGA_I2C_SDAT),
	.CLOCK_50					(CLOCK_50),
	.reset						(~KEY[0])
);

endmodule

