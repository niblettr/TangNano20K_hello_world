module uart #(
    parameter CLOCK_FREQUENCY = 27000000, // System clock frequency in Hz
    parameter BAUD_RATE = 115200          // UART baud rate
)(
    input       clk,                  // System clock
    input       start_uart_tx,        // Signal to enqueue data into the UART FIFO
    input [7:0] uart_tx_data,         // Data byte to transmit
    output reg  uart_tx_pin,          // UART transmit line
    output reg  uart_tx_fifo_ready,   // Indicates if the FIFO can accept more data

    input       uart_rx_pin,          // UART receive line

    output reg  rx_fifo_empty,    
    output reg  [7:0] rx_fifo_data_out,
    input       rx_fifo_read_en,

    output reg  Debug_uart     // Routed to pin in top module, for debug purposes...
);

    // Calculate the baud rate divisor
    localparam BAUD_DIVISOR = CLOCK_FREQUENCY / BAUD_RATE;
    localparam FIFO_SIZE_TX = 64; // FIFO size as a local parameter

    // Internal registers for transmission
    reg [15:0] tx_baud_counter = 0;          // Counter for baud rate timing
    reg [3:0] bit_index = 0;              // Index for bits being transmitted
    reg [9:0] tx_shift_reg = 10'b1111111111; // Shift register for start, data, and stop bits
    reg transmitting = 1'b0;              // Indicates if UART is currently transmitting

    // FIFO TX
    reg [7:0] tx_fifo [0:FIFO_SIZE_TX-1];       // FIFO buffer
    reg [5:0] tx_fifo_head = 0;              // Points to the next byte to transmit (6 bits for 64 entries)
    reg [5:0] tx_fifo_tail = 0;              // Points to the next free slot (6 bits for 64 entries)
    reg [6:0] tx_fifo_count = 0;             // Number of bytes in the FIFO (7 bits for counting up to 64)

    // Internal registers for reception
    reg [15:0] rx_baud_counter = 0;            // Counter for baud rate timing during reception
    reg [3:0]  rx_bit_index = 0;               // Index for bits being received
    reg [7:0]  rx_shift_reg = 8'b0;            // Shift register for receiving data
    reg        receiving = 1'b0;               // Indicates if UART is currently receiving
    reg uart_rx_pin_sync1, uart_rx_pin_sync2;  // Synchronize uart_rx_pin to the clock domain

    // FIFO RX
    reg rx_fifo_reset;
    reg rx_fifo_write_en;
    reg [7:0]rx_fifo_data_in;


/*
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
*/
fifo #(
    .DATA_WIDTH(8),
    .DEPTH(64)
) fifo_rx_inst (
    .clk(clk),
    .reset(rx_fifo_reset),
    .write_en(rx_fifo_write_en),
    .read_en(rx_fifo_read_en),
    .data_in(rx_fifo_data_in),
    .data_out(rx_fifo_data_out),
    .full(rx_fifo_full),
    .empty(rx_fifo_empty),
    .Debug_fifo(Debug_uart)
);

    // Initialize uart_tx_pin to idle state (high) and FIFO ready flag
// Initialize uart_tx_pin to idle state (high) and FIFO ready flag
reg [3:0] reset_counter = 4'b0; // 4-bit counter for reset delay
reg _reset = 1'b1;              // Reset signal
reg reset_done = 1'b0;          // Flag to indicate reset has been deasserted

always @(posedge clk) begin
    if (!reset_done) begin
        if (reset_counter < 4'd10) begin
            reset_counter <= reset_counter + 1'b1;
            _reset <= 1'b1; // Keep reset asserted
            rx_fifo_reset <= 1'b1;
        end else begin
            _reset <= 1'b0;        // Deassert reset after 10 clock cycles
            reset_done <= 1'b1;    // Latch that reset is done
            rx_fifo_reset <= 1'b0;
        end
    end
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
            tx_shift_reg <= {1'b1, tx_fifo[tx_fifo_head], 1'b0}; // Load start bit, data, and stop bit
            tx_fifo_head <= tx_fifo_head + 1'b1;              // Increment head pointer (wraps around)
            tx_fifo_count <= tx_fifo_count - 1'b1;            // Decrement FIFO count
            tx_baud_counter <= 0;                          // Reset baud counter
        end

        if (transmitting) begin
            if (tx_baud_counter < BAUD_DIVISOR - 1) begin
                tx_baud_counter <= tx_baud_counter + 1'b1;   // Increment baud counter
            end else begin
                tx_baud_counter <= 0;                     // Reset baud counter
                uart_tx_pin <= tx_shift_reg[0];           // Transmit the current bit
                tx_shift_reg <= {1'b1, tx_shift_reg[9:1]};   // Shift to the next bit
                bit_index <= bit_index + 1'b1;         // Increment bit index

                if (bit_index == 9) begin              // Stop after transmitting all bits
                    transmitting <= 1'b0;              // End transmission
                    bit_index <= 0;                    // Reset bit index
                end
            end
        end
    end

    /*********************************************************************************************/
    // UART Receive Logic

    // Synchronize uart_rx_pin to the clock domain
    always @(posedge clk) begin
        uart_rx_pin_sync1 <= uart_rx_pin;
        uart_rx_pin_sync2 <= uart_rx_pin_sync1;
    end

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
            if (rx_bit_index >= 1 && rx_bit_index <= 8) begin
                // Collect data bits (rx_bit_index 1 to 8)
                rx_shift_reg <= {uart_rx_pin_sync2, rx_shift_reg[7:1]};
            end

            rx_bit_index <= rx_bit_index + 1'b1;

            if (rx_bit_index == 9) begin
               // Stop bit received, validate it
               if (uart_rx_pin_sync2 && !rx_fifo_full) begin
                  rx_fifo_write_en <= 1'b1;          // Enable write to FIFO
                  rx_fifo_data_in <= rx_shift_reg;   // Write received data to FIFO
                  // Detect the character 'H' (ASCII 8'h48) and toggle Debug_uart
                  if (rx_shift_reg == 8'h48) begin
                      //Debug_uart <= ~Debug_uart;     // Toggle the debug line WORKING!!!!
                  end
               end
               receiving <= 1'b0;                    // End reception
            end
        end
    end else begin
        rx_fifo_write_en <= 1'b0; // Deassert write enable when not receiving
    end
end

endmodule