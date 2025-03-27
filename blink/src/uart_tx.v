module uart_tx #(
    parameter CLOCK_FREQUENCY = 27000000,
    parameter BAUD_RATE = 115200
)(
    input clk,
    input start,
    input [7:0] data,
    output reg tx,
    output reg ready // Indicates if the FIFO can accept more data
);

    localparam BAUD_DIVISOR = CLOCK_FREQUENCY / BAUD_RATE;
    reg [15:0] baud_counter = 0;
    reg [3:0] bit_index = 0;
    reg [9:0] shift_reg = 10'b1111111111;
    reg transmitting = 1'b0;

    // FIFO buffer
    reg [7:0] fifo [0:15]; // 16-byte FIFO
    reg [3:0] fifo_head = 0; // Points to the next byte to transmit
    reg [3:0] fifo_tail = 0; // Points to the next free slot
    reg [4:0] fifo_count = 0; // Number of bytes in the FIFO

    // Initialize tx to idle state (high)
    initial begin
        tx = 1'b1;
        ready = 1'b1; // FIFO is initially empty
    end

    always @(posedge clk) begin
        // Handle new data input
        if (start && (fifo_count < 16)) begin // Prevent FIFO overflow
            fifo[fifo_tail] <= data;
            fifo_tail <= (fifo_tail + 1) & 4'b1111; // Wrap around
            fifo_count <= fifo_count + 1;
        end

        // Update ready flag
        ready <= (fifo_count < 16);

        // Handle UART transmission
        if (!transmitting && fifo_count > 0) begin
            transmitting <= 1'b1;
            shift_reg <= {1'b1, fifo[fifo_head], 1'b0}; // Start bit, data, stop bit
            fifo_head <= (fifo_head + 1) & 4'b1111; // Wrap around
            fifo_count <= fifo_count - 1;
            baud_counter <= 0; // Reset baud counter
        end

        if (transmitting) begin
            if (baud_counter < BAUD_DIVISOR - 1) begin
                baud_counter <= baud_counter + 1;
            end else begin
                baud_counter <= 0;
                tx <= shift_reg[0]; // Transmit the current bit
                shift_reg <= {1'b1, shift_reg[9:1]};
                bit_index <= bit_index + 1;

                if (bit_index == 9) begin // Stop after transmitting all bits
                    transmitting <= 1'b0;
                    bit_index <= 0;
                end
            end
        end
    end
endmodule