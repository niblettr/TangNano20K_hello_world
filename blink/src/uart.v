module uart #(
    parameter CLOCK_FREQUENCY = 27000000, // System clock frequency in Hz
    parameter BAUD_RATE = 115200          // UART baud rate
)(
    input       clock,                      // System clock
    input       reset,
    output reg  uart_tx_pin,
    input       uart_rx_pin,

    input [7:0] tx_fifo_data_in,
    input       tx_fifo_write_en,
    output      tx_fifo_full,

    output reg  rx_fifo_empty,    
    output reg  [7:0] rx_fifo_data_out,
    output reg  UartPacketReceived,
    input       rx_fifo_read_en
);

// Calculate the baud rate divisor
localparam BAUD_DIVISOR = CLOCK_FREQUENCY / BAUD_RATE;

// TX FIFO
reg      tx_fifo_read_en;
reg [7:0]tx_fifo_data_out;

fifo #(.DATA_WIDTH(8),
    .DEPTH(32)
) fifo_tx_inst (
    .clock(clock),
    .reset(reset),
    .write_en(tx_fifo_write_en),
    .read_en(tx_fifo_read_en),
    .data_in(tx_fifo_data_in),
    .data_out(tx_fifo_data_out),
    .full(tx_fifo_full),
    .empty(tx_fifo_empty)
    );

// RX FIFO
reg      rx_fifo_write_en;
reg [7:0]rx_fifo_data_in;

fifo #(
    .DATA_WIDTH(8),
    .DEPTH(32)
) fifo_rx_inst (
    .clock(clock),
    .reset(reset),
    .write_en(rx_fifo_write_en),
    .read_en(rx_fifo_read_en),
    .data_in(rx_fifo_data_in),
    .data_out(rx_fifo_data_out),
    .full(rx_fifo_full),
    .empty(rx_fifo_empty)
);


// Idle period tracking
    reg [15:0] idle_period_counter  = 0;
    reg idle_counter_started = 1'b0;
localparam BYTE_PERIOD = BAUD_DIVISOR * 10;  // 1 byte = 10 bits

// UART TX internals
reg [15:0] tx_baud_counter = 0;              // Baud rate divider
reg [3:0]  bit_index = 0;                    // Bit index (0–9)
reg [9:0]  tx_shift_reg = 10'b1111111111;    // Start, data, stop bits
reg        transmitting = 1'b0;              // TX in progress

    /*********************************************************************************************/
    always @(posedge clock) begin
    // Default assignments
    tx_fifo_read_en <= 1'b0;

        if (!transmitting) begin
        uart_tx_pin <= 1'b1;  // Idle high

        if (!tx_fifo_empty) begin
            // Start new transmission
            transmitting       <= 1'b1;
            tx_shift_reg       <= {1'b1, tx_fifo_data_out, 1'b0}; // {Stop, Data[7:0], Start}
            tx_fifo_read_en    <= 1'b1;
            tx_baud_counter    <= 0;
            bit_index          <= 0;
        end
    end else begin
        // Ongoing transmission
           if (tx_baud_counter < BAUD_DIVISOR - 1) begin
            tx_baud_counter <= tx_baud_counter + 1'b1;
           end else begin
            tx_baud_counter <= 0;
            uart_tx_pin     <= tx_shift_reg[0];         // Output current bit
            tx_shift_reg    <= {1'b1, tx_shift_reg[9:1]}; // Logical shift right
            bit_index       <= bit_index + 1'b1;

            if (bit_index == 9) begin
                transmitting <= 1'b0; // All bits sent
                bit_index    <= 0;
              end
           end
        end
    end

    // Internal registers for reception
    reg [15:0] rx_baud_counter = 0;            // Counter for baud rate timing during reception
    reg [7:0]  rx_bit_index = 0;               // Index for bits being received
    reg [7:0]  rx_shift_reg = 8'b0;            // Shift register for receiving data
    reg        receiving = 1'b0;               // Indicates if UART is currently receiving

    reg uart_rx_pin_sync1 = 1;
    reg uart_rx_pin_sync2 = 1;  //Synchronize uart_rx_pin to the clock domain

    /*********************************************************************************************/
    // UART Receive Logic
    // Synchronize uart_rx_pin to the clock domain
    always @(posedge clock) begin
        uart_rx_pin_sync1 <= uart_rx_pin;
        uart_rx_pin_sync2 <= uart_rx_pin_sync1;
        UartPacketReceived <= 1'b0;
    // Start bit detection
    if (!receiving && !uart_rx_pin_sync2 ) begin
        receiving <= 1'b1;
        rx_baud_counter <= BAUD_DIVISOR / 2; // Start sampling in the middle of the start bit
        rx_bit_index <= 0;
        idle_counter_started <= 1'b0;        
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
                idle_period_counter <= 0;             // Reset idle_period_counter
                idle_counter_started <= 1'b1;
            end
        end

    end else begin
        rx_fifo_write_en <= 1'b0; // Deassert write enable when not receiving
        // Increment idle_period_counter when not receiving
        if (idle_counter_started) begin
            if (idle_period_counter < BYTE_PERIOD) begin
                idle_period_counter <= idle_period_counter + 1'b1;
            end else begin
                // idle_period detected
                UartPacketReceived   <= 1'b1; // Assert UartPacketReceived
                idle_period_counter  <= 0;    // Reset the counter
                idle_counter_started <= 1'b0;
            end
        end
    end
end

endmodule