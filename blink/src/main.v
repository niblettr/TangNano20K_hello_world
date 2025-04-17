module Top_module( 
    input  clock,         // System Clock 27MHz            Pin4
    input  SPI_SCK,       // SPI clock                     Pin52 (Nucleo PA5)
    input  SPI_CS,        // SPI chip select               Pin31 (Nucleo PB4)
    input  SPI_MOSI,      // SPI Master Out, Slave In      Pin71 (Nucleo PA7)
    output SPI_MISO,      // SPI Master In, Slave Out      Pin53 (Nucleo PA6)
    output [5:0] leds,    // Array for LEDs                Pin15,Pin16,Pin17,Pin18,Pin19,Pin20
    output Uart_TX_Pin,   // Transmit pin of UART          Pin55
    input  Uart_RX_Pin,   // Receive pin of UART           Pin49
    output Debug_Pin      // Debug toggle                  Pin30
);

    reg TopLevelDebug = 0;

    /********** Constants **********/
    parameter CLOCK_FREQUENCY = 27000000;  // 27 MHz crystal oscillator
    parameter HALF_PERIOD     = 100;       // Adjust for desired speed
    parameter integer LED_COUNT_DELAY = ((CLOCK_FREQUENCY / 1000) * HALF_PERIOD) - 1;

    parameter BAUD_RATE = 115200;          // Uart Baud Rate (tested up to 2Mb/s - can go way higer)

    /********** UART debug String **********/
    reg [7:0] uart_string [0:6] = {"H", "t", "e", "s", "t", 13, 10}; // "Test\r\n"
    parameter uart_string_len   = 7;
    reg [3:0] uart_string_index = 0; // Index for string transmission


    /********** UART Transmission **********/
    reg tx_fifo_write_en;
    reg [7:0] tx_fifo_data_in = 8'b0;     // Data to transmit
    /********** UART Reception **********/
    reg rx_fifo_read_en;
    reg [7:0] rx_fifo_data_out = 8'b0;
    /********** SPI Slave **********/
    wire [7:0] spi_fifo_data_out;            // Data received from SPI master
    reg        spi_fifo_read_en;
    //reg [7:0] spi_data_to_send = 8'b0; // Data to send back to SPI master NOT USED ATM

/**************************************************************************************************************/

    // led module instantiation
    led_scroll #(
        .CLOCK_FREQUENCY(CLOCK_FREQUENCY),
        .LED_COUNT_DELAY(LED_COUNT_DELAY)
    ) led_scroll_inst (
        .clock(clock),
        .leds(leds)
    );

    spi_slave spi_inst (
        .clock(clock),
        .spi_clk(SPI_SCK),
        .spi_cs(SPI_CS),
        .mosi(SPI_MOSI),
        .miso(SPI_MISO),
        .spi_fifo_data_out(spi_fifo_data_out),
        .spi_fifo_empty(spi_fifo_empty),
        .spi_fifo_read_en(spi_fifo_read_en),
        .data_to_send(spi_data_to_send), // not used....
        .Debug_spi(Debug_spi)
    );

    uart #(
        .CLOCK_FREQUENCY(CLOCK_FREQUENCY),
        .BAUD_RATE(BAUD_RATE)
    ) uart_inst (
        .clock(clock),
        .tx_fifo_data_in(tx_fifo_data_in),
        .uart_tx_pin(Uart_TX_Pin),
        //.tx_fifo_empty(tx_fifo_empty),
        .tx_fifo_write_en(tx_fifo_write_en),

        .uart_rx_pin(Uart_RX_Pin),
        .rx_fifo_empty(rx_fifo_empty),       // Connect rx_fifo_empty
        .rx_fifo_data_out(rx_fifo_data_out), // Connect RX FIFO data output
        .rx_fifo_read_en(rx_fifo_read_en),   // Connect RX FIFO read enable

        .Debug_uart(Debug_uart)
    );

/*
/****************************************************************************************************/
// test statemachine just for printing "Test" string out the uart
/*
reg [2:0]  uart_tx_state  = 3'b000; // State machine for UART string transmission
reg [32:0] wait_delay = 32'b0;
always @(posedge clock) begin

    case (uart_tx_state)
        3'b000: begin
           if (uart_string_index < uart_string_len) begin
              tx_fifo_data_in <= uart_string[uart_string_index]; // Load the current character
              tx_fifo_write_en <= 1'b1;                          // Trigger UART transmission
              uart_string_index <= uart_string_index + 1'b1;     // Move to the next character
              uart_tx_state <= 3'b010;
           end else begin
              uart_string_index <= 1'b0;     // reset index
              uart_tx_state <= 3'b011;       // Move to the wait state
           end
        end
        3'b010: begin
           tx_fifo_write_en <= 1'b0;         // Deassert start signal one clock cycle later
           uart_tx_state <= 3'b000;          // move back to start
        end

        3'b011: begin
            tx_fifo_write_en <= 1'b0;            // Deassert start signal
            if(wait_delay == 32'd2700000) begin  // 100ms
                wait_delay <= 32'b0;             // Reset wait delay
                uart_tx_state <= 3'b000;         // Go back to the first state
            end else begin
                wait_delay <= wait_delay + 1'b1; // Increment wait delay
            end
        end
    endcase
end
*/

/*****************************************************************************************/

always @(posedge clock) begin
    // Default assignments
    spi_fifo_read_en <= 1'b0;         // Deassert SPI FIFO read enable
    rx_fifo_read_en  <= 1'b0;         // Deassert UART RX FIFO read enable
    tx_fifo_write_en <= 1'b0;         // Deassert UART TX FIFO write enable

    // Echo SPI received data back over the UART
    if (!spi_fifo_empty && !spi_fifo_read_en) begin
       spi_fifo_read_en <= 1'b1;                // Assert read enable to read from SPI FIFO
       tx_fifo_write_en <= 1'b1;                // Trigger UART transmission
       tx_fifo_data_in <= spi_fifo_data_out;    // Load SPI FIFO data into UART TX FIFO
   end

    // Echo Uart RX data back out the Uart TX using the fifo
    if (!rx_fifo_empty && !rx_fifo_read_en) begin  // !rx_fifo_read_en gets rid of weird echo...FIX ASAP!!!
        rx_fifo_read_en  <= 1'b1;                  // Assert read enable to read from RX FIFO
        tx_fifo_write_en <= 1'b1;
        tx_fifo_data_in <= rx_fifo_data_out;       // Load RX FIFO data into UART TX
    end
end


/********** Continuous Assignment **********/
//assign Debug_Pin = Debug_uart;
assign Debug_Pin = Debug_spi;
//assign Debug_Pin = TopLevelDebug;

endmodule