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
    wire uart_fifo_ready;                // Indicates if FIFO can accept more data

    uart_tx #(
        .CLOCK_FREQUENCY(CLOCK_FREQUENCY),
        .BAUD_RATE(BAUD_RATE)
    ) uart_inst (
        .clk(Clock),
        .start_uart(start_uart),
        .uart_tx_data(uart_tx_data),
        .uart_tx_pin(Uart_TX_Pin),
        .fifo_ready(uart_fifo_ready)
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
reg [2:0] uart_state = 3'b00; // State machine for UART string transmission
reg [32:0] wait_delay = 32'b0;
always @(posedge Clock) begin
    case (uart_state)
        2'b00: begin
            if (uart_fifo_ready) begin
                uart_tx_data <= uart_string[uart_string_index]; // Load the current character
                start_uart <= 1'b1;   // Trigger UART transmission
                uart_state <= 3'b001; // Move to the next state
            end
        end
        2'b01: begin
            start_uart <= 1'b0;   // Deassert start signal
            uart_state <= 2'b010; // Move to the next state
        end
        2'b10: begin
            if (uart_fifo_ready) begin
                if (uart_string_index < uart_string_len-1) begin
                    uart_string_index <= uart_string_index + 1'b1; // Move to the next character
                    uart_state <= 3'b00; // Go back to the first state
                end else begin
                    uart_string_index <= 0; // Reset the index
                    uart_state <= 3'b011;   // Move to the next state
                end
            end
        end
        2'b011: begin
            if(wait_delay == 32'd2700000) begin  // 100ms
                uart_state <= 3'b000; // Go back to the first state
                wait_delay <= 32'b0; // Reset wait delay
            end else begin
                wait_delay <= wait_delay + 1'b1; // Increment wait delay
            end

        end
    endcase
end
*/

reg [2:0] uart_state  = 3'b000; // State machine for UART string transmission
reg [32:0] wait_delay = 32'b0;
always @(posedge Clock) begin

    case (uart_state)
        3'b000: begin
                if (uart_string_index < uart_string_len-1) begin
                    uart_tx_data <= uart_string[uart_string_index]; // Load the current character
                    uart_string_index <= uart_string_index + 1'b1; // Move to the next character
                    
                end else begin
                    uart_string_index <= 1'b0;  // reset index
                    start_uart <= 1'b1;         // Trigger UART transmission
                    uart_state <= 3'b010;       // Move to the next state
                end
        end

        3'b010: begin
            start_uart <= 1'b0;   // Deassert start signal
            if(wait_delay == 32'd2700000) begin  // 100ms
                wait_delay <= 32'b0; // Reset wait delay
                uart_state <= 3'b000; // Go back to the first state
            end else begin
                wait_delay <= wait_delay + 1'b1; // Increment wait delay
            end

        end
    endcase
end


/*****************************************************************************************/
/* Echo SPI received data back over the uart */
/*
reg [1:0] uart_spi_state = 2'b00; // State machine for UART transmission

always @(posedge Clock) begin
    case (uart_spi_state)
        2'b00: begin
                if (spi_data_ready && uart_fifo_ready) begin
                uart_tx_data <= spi_rx_data;    // Load SPI data into UART FIFO
                    //uart_tx_data <= 8'hAA;           // test uart byte
                start_uart <= 1'b1;             // Trigger UART FIFO enqueue
                spi_read_ack <= 1'b1;           // Acknowledge SPI data
                uart_spi_state <= 2'b01;        // Move to the next state
                end else begin
                    start_uart <= 1'b0;             // Ensure UART start is deasserted
                    spi_read_ack <= 1'b0;           // Ensure SPI acknowledgment is deasserted
            end
        end
        2'b01: begin
                start_uart <= 1'b0;      // Deassert start signal
                spi_read_ack <= 1'b0;    // Deassert SPI acknowledgment
                uart_spi_state <= 2'b10; // Wait for UART to finish transmission
        end
        2'b10: begin
                if (uart_fifo_ready) begin
                uart_spi_state <= 2'b00; // Go back to the first state
            end
        end
    endcase
end
*/
/*****************************************************************************************/

/********** SPI compare stuff  **********/
reg [7:0] spi_buffer [0:15];       // 16-byte buffer for SPI data
reg [4:0] spi_write_ptr = 0;       // Write pointer for the buffer
reg compare_buffer_full = 1'b0;    // Indicates if the buffer is full buffer_full
reg [7:0] fixed_buffer [0:15];     // Fixed buffer for comparison
reg match_flag  = 1'b0;            // Flag to indicate we have a 16 byte 
reg debug_match = 1'b0;            // Flag to indicate if buffers match


// Initialize the fixed buffer with a predefined pattern
//"SPI debug data"
//"SPI debug data to Uart brige with 64byte fifo\r\n"
initial begin
    {fixed_buffer[0],  fixed_buffer[1],  fixed_buffer[2],  fixed_buffer[3], 
     fixed_buffer[4],  fixed_buffer[5],  fixed_buffer[6],  fixed_buffer[7], 
     fixed_buffer[8],  fixed_buffer[9],  fixed_buffer[10], fixed_buffer[11], 
     fixed_buffer[12], fixed_buffer[13], fixed_buffer[14], fixed_buffer[15]} = 
    {"SPI debug data", 8'h0D, 8'h0A};

    for (integer i = 0; i < 16; i = i + 1) begin
        spi_buffer[i] = 8'b0; // Initialize SPI buffer to zero
    end
end

// verify 16 SPI bytes received with matching constant string of "SPI debug data\r\n"
// does not work.... HOW DAMN HARD CAN IT BE!!
/*
always @(posedge Clock) begin
    if (spi_data_ready && !compare_buffer_full) begin

        spi_buffer[spi_write_ptr] <= spi_rx_data;      // Store received data in buffer
        //spi_read_ack <= 1'b1;                          // Acknowledge SPI data
        spi_write_ptr <= spi_write_ptr + 1'b1;         // Increment write pointer

        if (spi_write_ptr == 5'd15) begin
            debug_match <=1;
            compare_buffer_full <= 1'b1;               // Mark buffer as full
        end
    end else begin
      //spi_read_ack <= 1'b0;
      end

    if (compare_buffer_full) begin
        match_flag = 1'b1; // Assume match initially
        
        // Compare spi_buffer with fixed_buffer
        for (integer i = 0; i < 16; i = i + 1) begin
            if (spi_buffer[i] != fixed_buffer[i]) begin
                match_flag = 1'b0; // Set match flag to 0 if mismatch
            end
        end

        compare_buffer_full <= 1'b0;  // Clear buffer full flag after comparison
        spi_write_ptr <= 0;           // Reset write pointer after processing
    end

    if (spi_write_ptr == 5'd15 && SPI_CS == 0) begin
        //debug_match <=1;
    end 

    if(SPI_CS == 1) begin
       debug_match <= 0; // clear it back down
    end
end
*/
/********** Continuous Assignment **********/
assign Debug_Pin = debug_match;
//assign Debug_Pin = Debug_spi;
endmodule