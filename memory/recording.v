`include "memory.v"

module recording (CLOCK_50, KEY, LEDR, active,
	ci, di, ei, fi, gi, ai, bi, csi, dsi, fsi, gsi, asi,
	co, do, eo, fo, go, ao, bo, cso, dso, fso, gso, aso);
	
input CLOCK_50, active;
input [2:0] KEY;
input ci, di, ei, fi, gi, ai, bi, csi, dsi, fsi, gsi, asi;

output [1:0] LEDR;
output co, do, eo, fo, go, ao, bo, cso, dso, fso, gso, aso;

memory c (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(ci),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(co),
	.recording(LEDR[0]),
	.playing(LEDR[1]),
);
memory d (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(di),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(do)
);
memory e (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(ei),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(eo)
);
memory f (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(fi),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(fo)
);
memory g (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(gi),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(go)
);
memory a (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(ai),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(ao)
);
memory b (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(bi),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(bo)
);

/////

memory cs (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(csi),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(cso)
);
memory ds (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(dsi),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(dso)
);
memory fs (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(fsi),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(fso)
);
memory gs (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(gsi),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(gso)
);
memory as (
	.reset(~KEY[0]),
	.clock(CLOCK_50),
	.in(asi),
	.record(~KEY[1] && active),
	.playback(~KEY[2] && active),
	.out(aso)
);

endmodule