module Top_module( 
    input  clock,          // System Clock 27MHz            Pin4
    input  SPI_SCK,        // SPI clock                     Pin52 (Nucleo PA5)
    input  SPI_CS,         // SPI chip select               Pin31 (Nucleo PB4)
    input  SPI_MOSI,       // SPI Master Out, Slave In      Pin71 (Nucleo PA7)
    output SPI_MISO,       // SPI Master In, Slave Out      Pin53 (Nucleo PA6)
    output [5:0] leds,     // Array for LEDs                Pin15,Pin16,Pin17,Pin18,Pin19,Pin20
    output Uart_TX_Pin,    // Transmit pin of UART          Pin55
    input  Uart_RX_Pin,    // Receive pin of UART           Pin49
    output Debug_Pin,      // Debug toggle                  Pin30
    output Debug_clock_pin // Debug toggle                  Pin29
);

    reg TopLevelDebug  = 0;
    reg TopLevelDebug2 = 0;

    /********** Constants **********/
    parameter CLOCK_FREQUENCY = 27000000;  // 27 MHz crystal oscillator
    parameter HALF_PERIOD     = 100;       // Adjust for desired speed
    parameter integer LED_COUNT_DELAY = ((CLOCK_FREQUENCY / 1000) * HALF_PERIOD) - 1;

    parameter BAUD_RATE = 115200;          // Uart Baud Rate (tested up to 2Mb/s - can go way higer)

    /********** UART debug String **********/
    reg [7:0] uart_string [0:5] = {"T", "e", "s", "t", 13, 10}; // "Test\r\n"
    parameter uart_string_len   = 6;
    reg [3:0] uart_string_index = 0; // Index for string transmission


    /********** UART Transmission **********/
    reg tx_fifo_write_en;
    reg [7:0] tx_fifo_data_in = 8'b0;     // Data to transmit
    /********** UART Reception **********/
    reg rx_fifo_read_en;
    reg [7:0] rx_fifo_data_out = 8'b0;
    wire rx_fifo_empty;
    /********** SPI Slave **********/
    wire [7:0] spi_fifo_data_out;            // Data received from SPI master
    reg        spi_fifo_read_en;
    //reg [7:0] spi_data_to_send = 8'b0; // Data to send back to SPI master NOT USED ATM
    wire spi_fifo_empty;
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
        //.data_to_send(spi_data_to_send), // not used....
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



/****************************************************************************************************/
// test statemachine just for printing "Test" string out the uart

reg [2:0] uart_tx_state = 3'b000; // State machine for UART string transmission
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


/*****************************************************************************************/
/*
reg uart_rx_processing = 1'b0; // Flag to track if RX processing is in progress
reg spi_rx_processing = 1'b0; // Flag to track if RX processing is in progress
always @(posedge clock) begin
    // Default assignments
    spi_fifo_read_en <= 1'b0;         // Deassert SPI FIFO read enable
    rx_fifo_read_en  <= 1'b0;         // Deassert UART RX FIFO read enable
    tx_fifo_write_en <= 1'b0;         // Deassert UART TX FIFO write enable

    // Echo SPI received data back over the UART

    if (!spi_fifo_empty && !spi_rx_processing) begin
       //TopLevelDebug <= ~TopLevelDebug;
       spi_fifo_read_en <= 1'b1;                // Assert read enable to read from SPI FIFO
       tx_fifo_write_en <= 1'b1;                // Trigger UART transmission
       spi_rx_processing <= 1'b1;                  // Set processing flag
       tx_fifo_data_in <= spi_fifo_data_out;    // Load SPI FIFO data into UART TX FIFO
    end else if (spi_fifo_empty) begin
        spi_rx_processing <= 1'b0;                  // Clear processing flag when FIFO is empty
    end

    // Echo Uart RX data back out the Uart TX using the fifo
    if (!rx_fifo_empty && !uart_rx_processing) begin  // !rx_fifo_read_en gets rid of weird echo...FIX ASAP!!!
        //TopLevelDebug <= ~TopLevelDebug;
        rx_fifo_read_en  <= 1'b1;                  // Assert read enable to read from RX FIFO
        tx_fifo_write_en <= 1'b1;
        uart_rx_processing <= 1'b1;                  // Set processing flag
        tx_fifo_data_in <= rx_fifo_data_out;       // Load RX FIFO data into UART TX
    end else if (rx_fifo_empty) begin
        uart_rx_processing <= 1'b0;                  // Clear processing flag when FIFO is empty
    end
end
*/



/*
    // Define FSM states with meaningful names
    typedef enum logic [2:0] {
        STATE_IDLE   = 3'b000,
        STATE_PARSE  = 3'b001,
        STATE_WAIT   = 3'b010,
        STATE_PASS   = 3'b011,
        STATE_FAIL   = 3'b100
    } state_t;


// Parameters for command handling

parameter CMD_LENGTH = 4; // Number of bytes in a command
reg [7:0] command_buffer [0:CMD_LENGTH-1]; // Buffer to store the command
reg [2:0] command_index = 0;               // Index for the command buffer
reg [2:0] command_state = STATE_IDLE;      // State machine for command handling

reg uart_rx_previous_empty = 1'b0; // Flag to track if RX processing is in progress

always @(posedge clock) begin
    // Default assignments
    rx_fifo_read_en <= 1'b0;         // Deassert UART RX FIFO read enable
    //tx_fifo_write_en <= 1'b0;        // Deassert UART TX FIFO write enable

    case (command_state)
        STATE_IDLE: begin
            // Idle state: Wait for data in RX FIFO
            if (!rx_fifo_empty && uart_rx_previous_empty) begin
                uart_rx_previous_empty <= 1'b0;
                rx_fifo_read_en <= 1'b1; // Read from RX FIFO
                command_buffer[command_index] <= rx_fifo_data_out; // Store received byte


                if (command_index == CMD_LENGTH -1 ) begin
                    command_index <= 3'b0;        // reset to zero
                    command_state <= STATE_PARSE; // Move to command processing state
                    //TopLevelDebug <= ~TopLevelDebug;
                end else begin
                   command_index <= command_index + 1'b1; // Increment buffer index
                end
            end else if (rx_fifo_empty) begin
              uart_rx_previous_empty <= 1'b1;
            end
        end

        STATE_PARSE: begin
            //// Command processing state
            if (command_buffer[0] == 8'h54 && command_buffer[1] == 8'h45 && command_buffer[2] == 8'h53 && command_buffer[3] == 8'h54) begin // Example: Command "TEST" 54 45 53 54
               command_state <= STATE_PASS;
            end else begin
              command_state <= STATE_FAIL;
            end
        end // case

        STATE_PASS: begin
           TopLevelDebug <= ~TopLevelDebug;
           command_state <= STATE_IDLE;
        end

        STATE_FAIL: begin
           TopLevelDebug2 <= ~TopLevelDebug2;
           command_state <= STATE_IDLE;
        end

        default: begin
            command_state <= STATE_IDLE; // Reset to idle state
        end
    endcase
end
*/

/********** Continuous Assignment **********/
assign Debug_clock_pin = TopLevelDebug2;
//assign Debug_Pin = Debug_uart;
//assign Debug_Pin = Debug_spi;
assign Debug_Pin = TopLevelDebug;

endmodule