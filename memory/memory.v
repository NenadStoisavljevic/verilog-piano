module memory (reset, clock, in, record, playback, out, recording, playing);

input reset, clock, in, record, playback;
output out, recording, playing;

wire recordDone, playDone;
wire [1:0] state;

fsm u0 (
    .reset(reset),
    .clock(clock),
    .record(record),
    .playback(playback),
    .state(state),
    .recordDone(recordDone),
    .playDone(playDone)
);

datapath_mem u1 (
    .clock(clock),
    .in(in),
    .state(state),
    .over(recordDone),
    .out(out),
    .done(playDone)
);

assign recording = (state == 2);
assign playing = (state == 3);
endmodule

module fsm(reset, clock, record, playback, state, recordDone, playDone);
input reset, clock, record, playback, recordDone, playDone;
output reg [1:0] state;

//states: 0: clear, 1: waiting, 2: recording, 3: playback

always@(posedge clock)
begin
    if(reset) state <= 0;
    else
    case(state)
        0: state <= 1;
        1: state <= playback ? 3 : (record ? 2 : 1);
        2: state <= recordDone ? 1 : 2;
        3: state <= playDone ? 1 : 3;
    endcase
end
endmodule

module datapath_mem (clock, in, state, over, out, done);
input clock, in;
input [1:0] state;
output reg out, over, done;

reg [31:0] storage [49:0];
wire [31:0] count = 32'd400000000;

reg [31:0] counter;
reg store;
integer i;
integer n = 0;

always@(posedge clock)
begin
    if (state == 0) begin
        counter <= 0;
        for (i=0; i<50; i=i+1) storage[i] <= 32'd0;
        store <= 0;
        n <= 0;
        out <= 0;
        over <= 0;
        done <= 0;
    end
    else if (state == 2) begin
        if (counter == count) begin
            counter <= count;
            n <= 0;
            store <= 0;
            over <= 1;
        end 
        else if (counter == 0) begin
            counter <= counter + 1;
        end
        else begin
            counter <= counter + 1;
            store <= in;
            if (in == ~store) begin
                storage[n] <= counter;
                n <= n+1;
            end
        end
    end
    else if (state == 3) begin
        if (counter == count) begin
            counter <= count;
            n <= 0;
            out <= 0;
            done <= 1;
        end
        else if (counter == 0) begin
            counter <= counter + 1;
        end
        else if (counter == storage[n]) begin
            counter <= counter + 1;
            n <= n + 1;
            out <= !out;
        end
        else begin
            counter <= counter + 1;
            n <= n;
            out <= out;
        end
    end
    else begin
        counter <= 0;
        store <= 0;
        out <= 0;
        n <= 0;
        over <= 0;
        done <= 0;
    end
end
endmodule