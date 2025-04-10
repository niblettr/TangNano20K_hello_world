module uart #(
    parameter CLOCK_FREQUENCY = 27000000, // System clock frequency in Hz
    parameter BAUD_RATE = 115200          // UART baud rate
)(
    input       clk,            // System clock
    input       start_uart_tx,     // Signal to enqueue data into the UART FIFO
    input [7:0] uart_tx_data,   // Data byte to transmit
    output reg  uart_tx_pin,    // UART transmit line
    input       uart_rx_pin,    // UART receive line
    output reg  uart_tx_fifo_ready,     // Indicates if the FIFO can accept more data
    output reg  Debug_uart,     // Routed to pin in top module, for debug purposes...
    output reg  [7:0] uart_rx_data, // Received data byte
    output reg  uart_rx_ready      // Indicates a byte has been received
);

    // Calculate the baud rate divisor
    localparam BAUD_DIVISOR = CLOCK_FREQUENCY / BAUD_RATE;
    localparam FIFO_SIZE_TX = 64; // FIFO size as a local parameter

    // Internal registers for transmission
    reg [15:0] baud_counter = 0;          // Counter for baud rate timing
    reg [3:0] bit_index = 0;              // Index for bits being transmitted
    reg [9:0] shift_reg = 10'b1111111111; // Shift register for start, data, and stop bits
    reg transmitting = 1'b0;              // Indicates if UART is currently transmitting

    // FIFO buffer for data
    reg [7:0] tx_fifo [0:FIFO_SIZE_TX-1];       // FIFO buffer
    reg [5:0] tx_fifo_head = 0;              // Points to the next byte to transmit (6 bits for 64 entries)
    reg [5:0] tx_fifo_tail = 0;              // Points to the next free slot (6 bits for 64 entries)
    reg [6:0] tx_fifo_count = 0;             // Number of bytes in the FIFO (7 bits for counting up to 64)

    // Internal registers for reception
    reg [15:0] rx_baud_counter = 0;       // Counter for baud rate timing during reception
    reg [3:0] rx_bit_index = 0;           // Index for bits being received
    reg [9:0] rx_shift_reg = 10'b0;       // Shift register for receiving data
    reg receiving = 1'b0;                 // Indicates if UART is currently receiving
    reg uart_rx_pin_sync1, uart_rx_pin_sync2; // Synchronize uart_rx_pin to the clock domain



fifo #(
    .DATA_WIDTH(8),
    .DEPTH(16)
) fifo_tx_inst (
    .clk(clk),
    .reset(tx_fifo_reset),
    .write_en(tx_fifo_write_en),
    .read_en(tx_fifo_read_en),
    .data_in(tx_fifo_data_in),
    .data_out(tx_fifo_data_out),
    .full(tx_fifo_full),
    .empty(tx_fifo_empty)
);

fifo #(
    .DATA_WIDTH(8),
    .DEPTH(16)
) fifo_rx_inst (
    .clk(clk),
    .reset(rx_fifo_reset),
    .write_en(rx_fifo_write_en),
    .read_en(rx_fifo_read_en),
    .data_in(rx_fifo_data_in),
    .data_out(rx_fifo_data_out),
    .full(rx_fifo_full),
    .empty(rx_fifo_empty)
);

    // Initialize uart_tx_pin to idle state (high) and FIFO ready flag
    initial begin
        uart_tx_pin = 1'b1;       // UART idle state
        uart_tx_fifo_ready  = 1'b1;       // FIFO is initially empty
        uart_rx_ready = 1'b0;     // No data received initially
    end

    // Synchronize uart_rx_pin to the clock domain
    always @(posedge clk) begin
        uart_rx_pin_sync1 <= uart_rx_pin;
        uart_rx_pin_sync2 <= uart_rx_pin_sync1;
    end

    // UART Transmit Logic
    always @(posedge clk) begin
        // Handle new data input
        if (start_uart_tx && (tx_fifo_count < FIFO_SIZE_TX)) begin
            tx_fifo[tx_fifo_tail] <= uart_tx_data;            // Store data in FIFO
            tx_fifo_tail <= tx_fifo_tail + 1'b1;              // Increment tail pointer (wraps around)
            tx_fifo_count <= tx_fifo_count + 1'b1;            // Increment FIFO count
        end
        
        uart_tx_fifo_ready <= (tx_fifo_count < FIFO_SIZE_TX);         // Update ready flag

        // Handle UART transmission
        if (!transmitting && tx_fifo_count > 0) begin
            transmitting <= 1'b1;                       // Start transmission
            shift_reg <= {1'b1, tx_fifo[tx_fifo_head], 1'b0}; // Load start bit, data, and stop bit
            tx_fifo_head <= tx_fifo_head + 1'b1;              // Increment head pointer (wraps around)
            tx_fifo_count <= tx_fifo_count - 1'b1;            // Decrement FIFO count
            baud_counter <= 0;                          // Reset baud counter
        end

        if (transmitting) begin
            if (baud_counter < BAUD_DIVISOR - 1) begin
                baud_counter <= baud_counter + 1'b1;   // Increment baud counter
            end else begin
                baud_counter <= 0;                     // Reset baud counter
                uart_tx_pin <= shift_reg[0];           // Transmit the current bit
                shift_reg <= {1'b1, shift_reg[9:1]};   // Shift to the next bit
                bit_index <= bit_index + 1'b1;         // Increment bit index

                if (bit_index == 9) begin              // Stop after transmitting all bits
                    transmitting <= 1'b0;              // End transmission
                    bit_index <= 0;                    // Reset bit index
                end
            end
        end
    end

    // UART Receive Logic
    always @(posedge clk) begin
        if (!receiving && !uart_rx_pin_sync2) begin
            // Start bit detected
            receiving <= 1'b1;
            rx_baud_counter <= BAUD_DIVISOR / 2; // Start sampling in the middle of the start bit
            rx_bit_index <= 0;
        end

        if (receiving) begin
            if (rx_baud_counter < BAUD_DIVISOR - 1) begin
                rx_baud_counter <= rx_baud_counter + 1'b1;
            end else begin
                rx_baud_counter <= 0;
                rx_shift_reg <= {uart_rx_pin_sync2, rx_shift_reg[9:1]}; // Shift in the received bit
                rx_bit_index <= rx_bit_index + 1'b1;

                if (rx_bit_index == 9) begin
                    // Stop bit received, latch the data
                    uart_rx_data <= rx_shift_reg[8:1]; // Extract the data bits
                    uart_rx_ready <= 1'b1;            // Indicate data is ready
                    receiving <= 1'b0;                // End reception
                end
            end
        end

        // Clear uart_rx_ready when data is read
        if (uart_rx_ready && !receiving) begin
            uart_rx_ready <= 1'b0;
        end
    end
endmodule