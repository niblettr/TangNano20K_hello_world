module uart #(
    parameter CLOCK_FREQUENCY = 27000000, // System clock frequency in Hz
    parameter BAUD_RATE = 115200          // UART baud rate
)(
    input       clock,                      // System clock
    output reg  uart_tx_pin,
    input       uart_rx_pin,
    output reg  Debug_uart,               // Routed to pin in top module, for debug purposes...
    //output reg  Debug_uart2,               // Routed to pin in top module, for debug purposes...

    input [7:0] tx_fifo_data_in,
    input       tx_fifo_write_en,

    output reg  rx_fifo_empty,    
    output reg  [7:0] rx_fifo_data_out,
    output reg  rx_sentence_received,
    input       rx_fifo_read_en
);

    // Calculate the baud rate divisor
    localparam BAUD_DIVISOR = CLOCK_FREQUENCY / BAUD_RATE;

    // TX FIFO
    reg      tx_fifo_reset;
    reg      tx_fifo_read_en;
    reg [7:0]tx_fifo_data_out;
    wire     tx_fifo_full;
    wire     tx_fifo_empty;

    // RX FIFO
    reg      rx_fifo_reset;
    reg      rx_fifo_write_en;
    reg [7:0]rx_fifo_data_in;
    wire     rx_fifo_full;

    fifo #(
        .DATA_WIDTH(8),
        .DEPTH(64)
    ) fifo_tx_inst (
        .clock(clock),
        .reset(tx_fifo_reset),
        .write_en(tx_fifo_write_en),
        .read_en(tx_fifo_read_en),
        .data_in(tx_fifo_data_in),
        .data_out(tx_fifo_data_out),
        .full(tx_fifo_full),
        .empty(tx_fifo_empty)
        //.Debug_fifo(Debug_uart_dummy)
    );

    fifo #(
        .DATA_WIDTH(8),
        .DEPTH(64)
    ) fifo_rx_inst (
        .clock(clock),
        .reset(rx_fifo_reset),
        .write_en(rx_fifo_write_en),
        .read_en(rx_fifo_read_en),
        .data_in(rx_fifo_data_in),
        .data_out(rx_fifo_data_out),
        .full(rx_fifo_full),
        .empty(rx_fifo_empty)
        //.Debug_fifo(Debug_uart)
    );


    // Initialize uart_tx_pin to idle state (high) and FIFO ready flag
    reg [1:0] reset_counter = 4'b0; // 1-bit counter for reset delay
    reg _reset = 1'b1;              // Reset signal
    reg reset_done = 1'b0;          // Flag to indicate reset has been deasserted

    // Add a timer for quiet period detection
    reg [15:0] quiet_period_counter = 0; // Counter for quiet period
    localparam BYTE_PERIOD = BAUD_DIVISOR * 10; // Duration of 1 byte (10 bits at the current baud rate)


    // initialisation
    always @(posedge clock) begin
        if (!reset_done) begin
            if (reset_counter < 4'd1) begin
                reset_counter <= reset_counter + 1'b1;
                _reset <= 1'b1; // Keep reset asserted
                rx_fifo_reset <= 1'b1;
                tx_fifo_reset <= 1'b1;
                //Debug_uart <= 1'b0;
            end else begin
               _reset <= 1'b0;        // Deassert reset after 10 clock cycles
                reset_done <= 1'b1;    // Latch that reset is done
                rx_fifo_reset <= 1'b0;
                tx_fifo_reset <= 1'b0;
            end
        end
    end

    // Internal registers for transmission
    reg [15:0] tx_baud_counter = 0;          // Counter for baud rate timing
    reg [3:0] bit_index = 0;                 // Index for bits being transmitted
    reg [9:0] tx_shift_reg = 10'b1111111111; // Shift register for start, data, and stop bits
    reg transmitting = 1'b0;                 // Indicates if UART is currently transmitting

    /*********************************************************************************************/
    // UART Transmit Logic
    always @(posedge clock) begin
        tx_fifo_read_en <= 1'b0;                           // Deassert read enable

        if (!transmitting) begin
           uart_tx_pin = 1'b1;                             // idle High
        end

        if (!transmitting && !tx_fifo_empty) begin
           //Debug_uart <= ~Debug_uart;
           transmitting <= 1'b1;                           // Start transmission
           tx_shift_reg <= {1'b1, tx_fifo_data_out, 1'b0}; // Load start bit, data, and stop bit
           tx_fifo_read_en <= 1'b1;                        // Read from TX FIFO
           tx_baud_counter <= 0;                           // Reset baud counter
           bit_index <= 0;                                 // Reset bit index
        end

        if (transmitting) begin
           if (tx_baud_counter < BAUD_DIVISOR - 1) begin
              tx_baud_counter <= tx_baud_counter + 1'b1; // Increment baud counter
           end else begin
              tx_baud_counter <= 0;                       // Reset baud counter
              uart_tx_pin <= tx_shift_reg[0];             // Transmit the current bit
              tx_shift_reg <= {1'b1, tx_shift_reg[9:1]};  // Shift to the next bit
              bit_index <= bit_index + 1'b1;              // Increment bit index

              if (bit_index == 9) begin                   // Stop after transmitting all bits
                 transmitting <= 1'b0;                    // End transmission
                 bit_index <= 0;                          // Reset bit index
              end
           end
        end
    end

    // Internal registers for reception
    reg [15:0] rx_baud_counter = 0;            // Counter for baud rate timing during reception
    reg [3:0]  rx_bit_index = 0;               // Index for bits being received
    reg [7:0]  rx_shift_reg = 8'b0;            // Shift register for receiving data
    reg        receiving = 1'b0;               // Indicates if UART is currently receiving

    reg uart_rx_pin_sync1 = 1;
    reg uart_rx_pin_sync2 = 1;  //Synchronize uart_rx_pin to the clock domain

    // Synchronize uart_rx_pin to the clock domain
    always @(posedge clock) begin
        uart_rx_pin_sync1 <= uart_rx_pin;
        uart_rx_pin_sync2 <= uart_rx_pin_sync1;

    // Start bit detection
    if (!receiving && !uart_rx_pin_sync2 ) begin
        receiving <= 1'b1;
        rx_baud_counter <= BAUD_DIVISOR / 2; // Start sampling in the middle of the start bit
        rx_bit_index <= 0;
        quiet_period_counter <= 0; // Reset quiet period counter
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
               end
               receiving <= 1'b0;                    // End reception
            end
        end
    end else begin
        rx_fifo_write_en <= 1'b0; // Deassert write enable when not receiving
    end
end

endmodule