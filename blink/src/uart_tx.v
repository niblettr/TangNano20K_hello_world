module uart_tx #(
    parameter CLOCK_FREQUENCY = 27000000, // System clock frequency in Hz
    parameter BAUD_RATE = 115200          // UART baud rate
)(
    input clk,                   // System clock
    input start_uart,            // Signal to enqueue data into the UART FIFO
    input [7:0] data,            // Data byte to transmit
    output reg tx,               // UART transmit line
    output reg fifo_ready,       // Indicates if the FIFO can accept more data
    output reg fifo_overflow     // Indicates data was dropped due to FIFO full
);

    // Constants
    localparam BAUD_DIVISOR = CLOCK_FREQUENCY / BAUD_RATE;
    localparam FIFO_SIZE = 64;
    localparam FIFO_MASK = FIFO_SIZE - 1; // For pointer wrapping

    // UART transmit state
    reg [15:0] baud_counter = 0;          // Counter for baud rate timing
    reg [3:0] bit_index = 0;              // Bit index (start + 8 data + stop = 10 bits max)
    reg [9:0] shift_reg = 10'b1111111111; // Shift register for framing
    reg transmitting = 1'b0;              // Indicates if UART is currently transmitting

    // FIFO buffer
    reg [7:0] fifo [0:FIFO_SIZE-1];
    reg [5:0] fifo_head = 0;              // Read pointer
    reg [5:0] fifo_tail = 0;              // Write pointer
    reg [6:0] fifo_count = 0;             // FIFO fill level

    // Initial idle state
    initial begin
        tx = 1'b1;
        fifo_ready = 1'b1;
        fifo_overflow = 1'b0;
    end

    always @(posedge clk) begin
        // Reset overflow flag
        fifo_overflow <= 1'b0;

        // Handle new data input
        if (start_uart) begin
            if (fifo_count < FIFO_SIZE) begin
                fifo[fifo_tail] <= data;
                fifo_tail <= (fifo_tail + 1'b1) & FIFO_MASK;
                fifo_count <= fifo_count + 1'b1;
            end else begin
                fifo_overflow <= 1'b1; // FIFO full, data dropped
            end
        end

        // Update FIFO ready status
        fifo_ready <= (fifo_count < FIFO_SIZE);

        // Load new byte if idle and FIFO has data
        if (!transmitting && fifo_count > 0) begin
            shift_reg <= {1'b1, fifo[fifo_head], 1'b0}; // Stop, data, start
            fifo_head <= (fifo_head + 1'b1) & FIFO_MASK;
            fifo_count <= fifo_count - 1'b1;
            transmitting <= 1'b1;
            bit_index <= 0;
            baud_counter <= 0;
        end

        // Handle active transmission
        if (transmitting) begin
            if (baud_counter < BAUD_DIVISOR - 1) begin
                baud_counter <= baud_counter + 1'b1;
            end else begin
                baud_counter <= 0;

                tx <= shift_reg[0];                     // Send current bit
                shift_reg <= {1'b1, shift_reg[9:1]};    // Shift in 1s to pad stop bits
                bit_index <= bit_index + 1'b1;

                if (bit_index == 9) begin               // Finished 10 bits
                    transmitting <= 1'b0;
                end
            end
        end
    end
endmodule