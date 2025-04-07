module uart_tx #(
    parameter CLOCK_FREQUENCY = 27000000, // System clock frequency in Hz
    parameter BAUD_RATE = 115200          // UART baud rate
)(
    input clk,                // System clock
    input start_uart,         // Signal to enqueue data into the UART FIFO
    input [7:0] data,         // Data byte to transmit
    output reg tx,            // UART transmit line
    output reg fifo_ready          // Indicates if the FIFO can accept more data
);

    // Calculate the baud rate divisor
    localparam BAUD_DIVISOR = CLOCK_FREQUENCY / BAUD_RATE;

    // Internal registers
    reg [15:0] baud_counter = 0;          // Counter for baud rate timing
    reg [3:0] bit_index = 0;              // Index for bits being transmitted
    reg [9:0] shift_reg = 10'b1111111111; // Shift register for start, data, and stop bits
    reg transmitting = 1'b0;              // Indicates if UART is currently transmitting

    // FIFO buffer for data
    reg [7:0] fifo [0:15];                // 16-byte FIFO buffer
    reg [3:0] fifo_head = 0;              // Points to the next byte to transmit
    reg [3:0] fifo_tail = 0;              // Points to the next free slot
    reg [4:0] fifo_count = 0;             // Number of bytes in the FIFO


    // Initialize tx to idle state (high) and FIFO ready flag
    initial begin
        tx = 1'b1;       // UART idle state
        fifo_ready = 1'b1;    // FIFO is initially empty
    end

    always @(posedge clk) begin
        // Handle new data input
        if (start_uart && (fifo_count < 16)) begin // Prevent FIFO overflow
            fifo[fifo_tail] <= data;               // Store data in FIFO
            fifo_tail <= fifo_tail + 1'b1;         // Increment tail pointer (wraps around)
            fifo_count <= fifo_count + 1'b1;       // Increment FIFO count
        end

        // Update ready flag
        fifo_ready <= (fifo_count < 16);

        // Handle UART transmission
        if (!transmitting && fifo_count > 0) begin
            transmitting <= 1'b1;                       // Start transmission
            shift_reg <= {1'b1, fifo[fifo_head], 1'b0}; // Load start bit, data, and stop bit
            fifo_head <= fifo_head + 1'b1;              // Increment head pointer (wraps around)
            fifo_count <= fifo_count - 1'b1;            // Decrement FIFO count
            baud_counter <= 0;                          // Reset baud counter
        end

        if (transmitting) begin
            if (baud_counter < BAUD_DIVISOR - 1) begin
                baud_counter <= baud_counter + 1'b1; // Increment baud counter
            end else begin
                baud_counter <= 0;                  // Reset baud counter
                tx <= shift_reg[0];                 // Transmit the current bit
                shift_reg <= {1'b1, shift_reg[9:1]}; // Shift to the next bit
                bit_index <= bit_index + 1'b1;      // Increment bit index

                if (bit_index == 9) begin           // Stop after transmitting all bits
                    transmitting <= 1'b0;           // End transmission
                    bit_index <= 0;                 // Reset bit index
                end
            end
        end
    end
endmodule