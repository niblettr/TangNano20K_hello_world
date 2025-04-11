module Top_module( 
    input  Clock,         // System Clock 27MHz            Pin4
    input  SPI_SCK,       // SPI clock                     Pin52
    input  SPI_CS,        // SPI chip select               Pin31
    input  SPI_MOSI,      // SPI Master Out, Slave In      Pin71
    output SPI_MISO,      // SPI Master In, Slave Out      Pin53
    output [5:0] leds,    // Array for LEDs                Pin15,Pin16,Pin17,Pin18,Pin19,Pin20
    output Uart_TX_Pin,   // Transmit pin of UART          Pin55
    input  Uart_RX_Pin,   // Receive pin of UART           Pin51
    output Debug_Pin      // Debug toggle                  Pin30
);

    reg TopLevelDebug = 0;

    /********** Constants **********/
    parameter CLOCK_FREQUENCY = 27000000;  // 27 MHz crystal oscillator
    parameter HALF_PERIOD     = 100;       // Adjust for desired speed
    parameter integer LED_COUNT_DELAY = ((CLOCK_FREQUENCY / 1000) * HALF_PERIOD) - 1;

    parameter BAUD_RATE = 115200;
    //parameter UART_DELAY = CLOCK_FREQUENCY / 100; // 1-second delay for UART transmission

    /********** UART String **********/
    reg [7:0] uart_string [0:5] = {"t", "e", "s", "t", 13, 10}; // "Test\r\n"
    parameter uart_string_len   = 6;
    reg [3:0] uart_string_index = 0; // Index for string transmission


/**************************************************************************************************************/
    // led module instantiation
    led_scroll #(
        .CLOCK_FREQUENCY(CLOCK_FREQUENCY),
        .LED_COUNT_DELAY(LED_COUNT_DELAY)
    ) led_scroll_inst (
        .system_clk(Clock),
        .leds(leds)
    );


    /********** UART Transmission **********/
    reg [7:0] uart_tx_data = 8'b0;  // Data to transmit
    reg start_uart = 1'b0;          // Start signal for UART
    wire uart_fifo_ready;           // Indicates if FIFO can accept more data

    uart_tx #(
        .CLOCK_FREQUENCY(CLOCK_FREQUENCY),
        .BAUD_RATE(BAUD_RATE)
    ) uart_inst (
        .clk(Clock),
        .start_uart(start_uart),
        .uart_tx_data(uart_tx_data),
        .uart_tx_pin(Uart_TX_Pin),
        .fifo_ready(uart_fifo_ready),
        .Debug_uart(Debug_uart)
    );

    /********** SPI Slave **********/
    wire [7:0] spi_rx_data;            // Data received from SPI master
    wire spi_data_ready;               // Indicates SPI data is ready
    reg spi_read_ack = 1'b0;           // Acknowledge signal for SPI data
    reg [7:0] spi_data_to_send = 8'b0; // Data to send back to SPI master

    spi_slave spi_inst (
        .system_clk(Clock),
        .spi_clk(SPI_SCK),
        .spi_cs(SPI_CS),
        .mosi(SPI_MOSI),
        .miso(SPI_MISO),
        .spi_data_ready(spi_data_ready),
        .spi_read_ack(spi_read_ack),
        .spi_rx_data(spi_rx_data), 
        .data_to_send(spi_data_to_send),
        .Debug_spi(Debug_spi)
    );



/********** UART String Transmission **********/
// works flawlessly
/*
reg [2:0] uart_tx_state  = 3'b000; // State machine for UART string transmission
reg [32:0] wait_delay = 32'b0;
always @(posedge Clock) begin

    case (uart_tx_state)
        3'b000: begin
           if (uart_string_index < uart_string_len) begin
              uart_tx_data <= uart_string[uart_string_index]; // Load the current character
              start_uart <= 1'b1;                             // Trigger UART transmission
              uart_string_index <= uart_string_index + 1'b1;  // Move to the next character
              uart_tx_state <= 3'b010;
           end else begin
              uart_string_index <= 1'b0;     // reset index
              uart_tx_state <= 3'b011;       // Move to the wait state
           end
        end
        3'b010: begin
           start_uart <= 1'b0;               // Deassert start signal one clock cycle later
           uart_tx_state <= 3'b000;          // move back to start
        end

        3'b011: begin
            start_uart <= 1'b0;   // Deassert start signal
            if(wait_delay == 32'd2700000) begin  // 100ms
                wait_delay <= 32'b0; // Reset wait delay
                uart_tx_state <= 3'b000; // Go back to the first state
            end else begin
                wait_delay <= wait_delay + 1'b1; // Increment wait delay
            end

        end
    endcase
end
*/

/*****************************************************************************************/
/* Echo SPI received data back over the uart */
reg spi_data_processed = 1'b0; // Flag to ensure data is processed only once

always @(posedge Clock) begin
   //TopLevelDebug = ~TopLevelDebug;
   start_uart <= 1'b0;             // Ensure UART start is deasserted
   spi_read_ack <= 1'b0;           // Ensure SPI acknowledgment is deasserted

   if (spi_data_ready && !spi_data_processed) begin
      uart_tx_data <= spi_rx_data;    // Load SPI data into UART FIFO
      spi_read_ack <= 1'b1;           // Acknowledge SPI data, clears spi_data_ready in SPI module
      start_uart <= 1'b1;             // Trigger UART transmission
      spi_data_processed <= 1'b1;     // Mark data as processed
   end

   if (!spi_data_ready) begin
      spi_data_processed <= 1'b0;     // Reset flag when no data is ready
   end
end

/*****************************************************************************************/

/********** Continuous Assignment **********/
//assign Debug_Pin = Debug_uart;
//assign Debug_Pin = Debug_spi;
assign Debug_Pin = TopLevelDebug;
endmodule