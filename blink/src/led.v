module led( 
    input  Clock,         // System Clock 27MHz            Pin4
    input  SPI_SCK,       // SPI clock                     Pin52
    input  SPI_CS,        // SPI chip select               Pin31
    input  SPI_MOSI,      // SPI Master Out, Slave In      Pin71
    output SPI_MISO,      // SPI Master In, Slave Out      Pin53
    output [5:0] leds,    // Array for LEDs                Pin15,Pin16,Pin17,Pin18,Pin19,Pin20
    output Uart_TX,       // Transmit pin of UART          Pin55
    input  Uart_RX,       // Receive pin of UART           Pin51
    output Debug_Pin      // Debug toggle                  Pin30
);

    /********** UART String **********/
    reg [7:0] uart_string [0:5] = {"t", "e", "s", "a", 13, 10}; // "Test\r\n"
    reg [3:0] uart_string_index = 0; // Index for string transmission


    /********** Constants **********/
    parameter CLOCK_FREQUENCY = 27000000;  // 27 MHz crystal oscillator
    parameter HALF_PERIOD     = 100;       // Adjust for desired speed
    parameter integer COUNT_DELAY = ((CLOCK_FREQUENCY / 1000) * HALF_PERIOD) - 1;

    parameter BAUD_RATE = 115200;
    parameter BAUD_DIVISOR = CLOCK_FREQUENCY / BAUD_RATE;
    parameter UART_DELAY = CLOCK_FREQUENCY / 100; // 1-second delay for UART transmission

    /********** Counters **********/
    reg [23:0] count_value_reg = 0; // Counter for LED delay
    reg [31:0] uart_counter = 0;    // Counter for 1-second UART delay

    /********** LED State (Shift Register) **********/
    reg [5:0] led_state = 6'b000001; // Initial state: Only Led_0 is ON
    reg direction = 1'b1;           // 1 for right, 0 for left

    /********** UART Transmission **********/
    reg [7:0] uart_data = 8'b0;  // Data to transmit
    reg start_uart = 1'b0;       // Start signal for UART
    wire uart_ready;             // Indicates if FIFO can accept more data

    uart_tx #(
        .CLOCK_FREQUENCY(CLOCK_FREQUENCY),
        .BAUD_RATE(BAUD_RATE)
    ) uart_inst (
        .clk(Clock),
        .start_uart(start_uart),
        .data(uart_data),
        .tx(Uart_TX),
        .ready(uart_ready)
    );

    /********** SPI Slave **********/
    wire [7:0] spi_received_data;      // Data received from SPI master
    wire spi_data_ready;               // Indicates SPI data is ready
    reg spi_read_ack = 1'b0;           // Acknowledge signal for SPI data
    reg [7:0] spi_data_to_send = 8'b0; // Data to send back to SPI master

    spi_slave spi_inst (
        .system_clk(Clock),
        .spi_clk(SPI_SCK),
        .spi_cs(SPI_CS),
        .mosi(SPI_MOSI),
        .miso(SPI_MISO),
        .data_ready(spi_data_ready),
        .read_ack(spi_read_ack),
        .received_data(spi_received_data),
        .data_to_send(spi_data_to_send),
        .Debug(Debug_Pin)
    );

    /****************************************************************************************************/
    // LED handling
    
    always @(posedge Clock) begin
        // Counter for delay
        if (count_value_reg < COUNT_DELAY) begin
            count_value_reg <= count_value_reg + 1'b1; // Increment counter
        end
        else begin
            count_value_reg <= 0; // Reset counter

            // Shift LED state
            if (direction) begin
                if (led_state == 6'b100000) 
                    direction <= 1'b0; // Change direction to left
                else 
                    led_state <= led_state << 1; // Shift left
            end
            else begin
                if (led_state == 6'b000001) 
                    direction <= 1'b1; // Change direction to right
                else 
                    led_state <= led_state >> 1; // Shift right
            end
        end
    end

/********** UART String Transmission **********/
/*
reg [1:0] uart_state = 2'b00; // State machine for UART string transmission
always @(posedge Clock) begin
    case (uart_state)
        2'b00: begin
            if (uart_ready) begin
                uart_data <= uart_string[uart_string_index]; // Load the current character
                uart_start <= 1'b1; // Trigger UART transmission
                uart_state <= 2'b01; // Move to the next state
            end
        end
        2'b01: begin
            uart_start <= 1'b0; // Deassert start signal
            uart_state <= 2'b10; // Wait for UART to finish transmission
        end
        2'b10: begin
            if (uart_ready) begin
                if (uart_string_index < 5) begin
                    uart_string_index <= uart_string_index + 1'b1; // Move to the next character
                    uart_state <= 2'b00; // Go back to the first state
                end else begin
                    uart_string_index <= 0; // Reset the index
                    uart_state <= 2'b11; // Restart transmission if needed
                end
            end
        end
        2'b11: begin
           end
    endcase
end
*/

    reg [1:0] uart_spi_state = 2'b00; // State machine for UART transmission

    always @(posedge Clock) begin
        case (uart_spi_state)
            2'b00: begin
                if (spi_data_ready && uart_ready) begin
                    uart_data <= spi_received_data; // Load SPI data into UART FIFO
                    //uart_data <= 8'hAA;           // test uart byte
                    start_uart <= 1'b1;             // Trigger UART FIFO enqueue
                    spi_read_ack <= 1'b1;           // Acknowledge SPI data
                    uart_spi_state <= 2'b01;        // Move to the next state
                end else begin
                    start_uart <= 1'b0;             // Ensure UART start is deasserted
                    spi_read_ack <= 1'b0;           // Ensure SPI acknowledgment is deasserted
                end
            end
            2'b01: begin
                start_uart <= 1'b0; // Deassert start signal
                spi_read_ack <= 1'b0; // Deassert SPI acknowledgment
                uart_spi_state <= 2'b10; // Wait for UART to finish transmission
            end
            2'b10: begin
                if (uart_ready) begin
                    uart_spi_state <= 2'b00; // Go back to the first state
                end
            end
        endcase
    end


/********** Continuous Assignments **********/

generate
    genvar i; // Declare a generate variable 'i'
    for (i = 0; i < 6; i = i + 1) begin
        assign leds[i] = ~led_state[i]; // Invert because active low
    end
endgenerate

endmodule