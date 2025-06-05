/*
task hex_to_ascii;
    input  [7:0] in_bytes [0:3];   // Input byte array (4 bytes)
    input  integer num_bytes;      // Number of bytes to convert (should be 4)
    output [7:0] out_ascii [0:7];  // Output ASCII array (8 chars: 2 per byte)
    integer i;
    reg [7:0] high, low;
    begin
        for (i = 0; i < num_bytes; i = i + 1) begin
            high = (in_bytes[i][7:4] < 10) ? (in_bytes[i][7:4] + 8'd48) : (in_bytes[i][7:4] - 4'd10 + 8'd65);
            low  = (in_bytes[i][3:0] < 10) ? (in_bytes[i][3:0] + 8'd48) : (in_bytes[i][3:0] - 4'd10 + 8'd65);
            out_ascii[i*2]   = high;
            out_ascii[i*2+1] = low;
        end
    end
endtask
*/


function automatic logic [7:0] hex_to_ascii_nib(input logic [3:0] hex_value);

        return (hex_value < 10) ? (hex_value + 8'd48) : (hex_value - 4'd10 + 8'd65);

endfunction

function automatic void hex_to_ascii_32(
    input  logic [7:0] in_bytes [0:3],   // Input byte array (4 bytes)
    output logic [7:0] out_ascii [0:7]   // Output ASCII array (8 chars: 2 per byte)
);
    int i;
    logic [7:0] high, low;
    begin
        for (i = 0; i < 4; i++) begin
            high = (in_bytes[i][7:4] < 10) ? (in_bytes[i][7:4] + 8'd48) : (in_bytes[i][7:4] - 4'd10 + 8'd65);
            low  = (in_bytes[i][3:0] < 10) ? (in_bytes[i][3:0] + 8'd48) : (in_bytes[i][3:0] - 4'd10 + 8'd65);
            out_ascii[i*2]   = high;
            out_ascii[i*2+1] = low;
        end
    end
endfunction
