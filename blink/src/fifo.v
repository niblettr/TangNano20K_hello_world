module fifo #(
    parameter DATA_WIDTH = 8,  // Width of the data
    parameter DEPTH = 16       // Depth of the FIFO (number of entries)
)(
    input                   clock,        // Clock signal
    input                   reset,      // Reset signal (active high)
    input                   write_en,   // Write enable
    input                   read_en,    // Read enable
    input  [DATA_WIDTH-1:0] data_in,    // Data to write into the FIFO
    output [DATA_WIDTH-1:0] data_out,   // Data read from the FIFO
    output                  full,       // FIFO full flag
    output                  empty,      // FIFO empty flag
    output reg              Debug_fifo
);

    // Internal signals
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];   // Memory array for the FIFO
    reg [$clog2(DEPTH)-1:0] write_ptr = 0;  // Write pointer
    reg [$clog2(DEPTH):0] read_ptr = 0;     // Read pointer
    reg [$clog2(DEPTH):0] count = 0;        // Count of elements in the FIFO

    reg read_en_d;
    reg write_en_d;

    // Capture previous state of read_en & write_en
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            read_en_d  <= 1'b1; // Initialize during reset
            write_en_d <= 1'b1; // Initialize during reset
            Debug_fifo <= 1'b1;
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
           mem[write_ptr] <= data_in;       // Write data to memory
           write_ptr <= write_ptr + 1'b1;  // Increment write pointer
           // or write_ptr <= (write_ptr + 1'b1) % DEPTH;  // Increment write pointer
        end
    end

    // Read operation
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            read_ptr <= 0;
        end else if (read_en && !read_en_d && !empty) begin
            read_ptr <= read_ptr + 1'b1;    // Increment read pointer
            //or read_ptr <= (read_ptr + 1'b1) % DEPTH;    // Increment read pointer
            //Debug_fifo <= ~Debug_fifo;
        end
    end

    // Count management
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            count <= 0;
        end else begin
            case ({write_en && !write_en_d && !full, read_en && !read_en_d && !empty})
                2'b10: count <= count + 1'b1; // Write only
                2'b01: count <= count - 1'b1; // Read only
                default: count <= count;      // No change
            endcase
        end
    end

    // Output assignments
    assign data_out = mem[read_ptr];
    assign full = (count == DEPTH);
    assign empty = (count == 0);

endmodule