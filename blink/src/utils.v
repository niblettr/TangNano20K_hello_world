

// not tested yet
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

function automatic logic [3:0] ascii_hex_to_nibble(input logic [7:0] c);
         if (c >= "0" && c <= "9") return c - "0";
    else if (c >= "a" && c <= "f") return c - "a" + 4'd10;
    else if (c >= "A" && c <= "F") return c - "A" + 4'd10;
    else
        return 4'hF; // invalid nibble
endfunction

function automatic logic [7:0] hex_to_ascii_nib(input logic [3:0] hex_value);
    return (hex_value < 10) ? (hex_value + 8'd48) : (hex_value - 4'd10 + 8'd65);
endfunction
