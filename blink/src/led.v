module led( 
    input  Clock,
    output [5:0] leds,   // Array for LEDs (Pins 15-20)
    output Uart_TX,      // Pin49
    input  Uart_RX       // Pin55
);

    /********** UART String **********/
    reg [7:0] uart_string [0:5] = {"T", "i", "s", "s", 13, 10}; // "Test\r\n"
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
    reg uart_start = 1'b0;       // Start signal for UART
    wire uart_ready;             // Indicates if FIFO can accept more data

    uart_tx #(
        .CLOCK_FREQUENCY(CLOCK_FREQUENCY),
        .BAUD_RATE(BAUD_RATE)
    ) uart_inst (
        .clk(Clock),
        .start(uart_start),
        .data(uart_data),
        .tx(Uart_TX),
        .ready(uart_ready)
    );

    /****************************************************************************************************/
    // LED handling
    always @(posedge Clock) begin
        // Counter for delay
        if (count_value_reg < COUNT_DELAY) begin
            count_value_reg <= count_value_reg + 1; // Increment counter
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

    /****************************************************************************************************/
    // UART transmission logic using FIFO
    always @(posedge Clock) begin
        if (uart_counter < UART_DELAY) begin
            uart_counter <= uart_counter + 1; // Increment UART delay counter
            uart_start <= 1'b0; // Ensure start is low unless sending data
        end
        else if (uart_ready && !uart_start) begin
            uart_counter <= 0; // Reset UART delay counter
            uart_data <= uart_string[uart_string_index]; // Load the current character
            uart_start <= 1'b1; // Trigger UART transmission
            
            // Move to the next character
            uart_string_index <= (uart_string_index < 5) ? uart_string_index + 1 : 0;
        end
        else begin
            uart_start <= 1'b0; // Ensure `start` is deasserted after one cycle
        end
    end

    /********** Continuous Assignments **********/
generate
    genvar i; // Declare a generate variable 'i'
    for (i = 0; i < 6; i = i + 1) begin
        assign leds[i] = ~led_state[i]; // Invert because active low
    end
endgenerate

endmodule