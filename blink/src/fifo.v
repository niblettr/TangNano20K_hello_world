module fifo #(
    parameter DATA_WIDTH = 8,   // Width of the data
    parameter DEPTH      = 16   // Depth of the FIFO (number of entries)
)(
    input                   clock,      // Clock signal
    input                   reset,      // Reset signal (active high)
    input                   write_en,   // Write enable
    input                   read_en,    // Read enable
    input  [DATA_WIDTH-1:0] data_in,    // Data input
    output [DATA_WIDTH-1:0] data_out,   // Data output
    output                  full,       // FIFO full flag
    output                  empty       // FIFO empty flag
);

    // Internal memory and control signals
    reg [DATA_WIDTH-1:0] fifo_array [0:DEPTH-1];
    reg [$clog2(DEPTH)-1:0] write_ptr;
    reg [$clog2(DEPTH)-1:0] read_ptr;
    reg [$clog2(DEPTH):0]   count;

    reg read_en_d;
    reg write_en_d;

    // Previous state capture for edge detection
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            read_en_d  <= 1'b1;
            write_en_d <= 1'b1;
        end else begin
            read_en_d  <= read_en;
            write_en_d <= write_en;
        end
    end

    // Write operation
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            write_ptr <= 0;
        end else if (write_en && !write_en_d && !full) begin
            fifo_array[write_ptr] <= data_in;
            write_ptr <= write_ptr + 1'b1;
        end
    end

    // Read operation
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            read_ptr <= 0;
        end else if (read_en && !read_en_d && !empty) begin
            read_ptr <= read_ptr + 1'b1;
        end
    end

    // Count management
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            count <= 0;
        end else begin
            case ({write_en && !write_en_d && !full, read_en && !read_en_d && !empty})
                2'b10: count <= count + 1'b1;  // Write only
                2'b01: count <= count - 1'b1;  // Read only
                default: count <= count;       // No change
            endcase
        end
    end

    // Output assignments
    assign data_out = fifo_array[read_ptr];
    assign full     = (count == DEPTH);
    assign empty    = (count == 0);

endmodule