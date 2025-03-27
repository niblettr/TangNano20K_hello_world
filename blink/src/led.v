module led( 
    input  Clock,
    output Led_0,   // Pin15
    output Led_1,   // Pin16
    output Led_2,   // Pin17
    output Led_3,   // Pin18
    output Led_4,   // Pin19
    output Led_5,   // Pin20
    output Uart_TX, // Pin49
    input  Uart_RX  // Pin55
);

    /********** UART String **********/
    reg [7:0] uart_string [0:5] = {"T", "e", "s", "t", 13, 10};
    reg [3:0] uart_string_index = 0; // Index for string transmission    

    /********** Constants **********/
    parameter  CLOCK_FREQUENCY = 27000000;  // 27 MHz crystal oscillator
    parameter  HALF_PERIOD     = 100;       // Adjust for desired speed
    parameter  integer COUNT_DELAY = ((CLOCK_FREQUENCY / 1000) * HALF_PERIOD) - 1;
    parameter  BAUD_RATE = 115200;
    parameter  BAUD_DIVISOR = CLOCK_FREQUENCY / BAUD_RATE;
    parameter  UART_DELAY = CLOCK_FREQUENCY / 100; // 1-second delay for UART transmission

    /********** Counters **********/
    reg [23:0] count_value_reg = 0; // Counter register (24 bits)
    reg [31:0] uart_counter = 0;    // Counter for 1-second UART delay

    /********** LED State **********/
    reg [5:0] led_state = 6'b000001; // Initial state: Only Led_0 is ON
    reg direction = 1'b1;           // 1 for right, 0 for left

    /********** UART Transmission **********/
    reg [7:0] uart_data = 8'b0;     // Data to transmit
    reg uart_start = 1'b0;          // Start signal for UART
    wire uart_ready;                // Indicates if FIFO can accept more data

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

            // Update LED state
            if (direction) begin
                if (led_state == 6'b100000) begin
                    direction <= 1'b0; // Change direction to left
                end
                else begin
                    led_state <= led_state << 1; // Shift left
                end
            end
            else begin
                if (led_state == 6'b000001) begin
                    direction <= 1'b1; // Change direction to right
                end
                else begin
                    led_state <= led_state >> 1; // Shift right
                end
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
            if (uart_string_index < 5) begin
                uart_string_index <= uart_string_index + 1;
            end
            else begin
                uart_string_index <= 0; // Reset index after the last character
            end
        end
        else begin
            uart_start <= 1'b0; // Ensure `start` is deasserted after one cycle
        end
    end

    /********** Continuous Assignments **********/
    assign Led_0 = !led_state[0];
    assign Led_1 = !led_state[1];
    assign Led_2 = !led_state[2];
    assign Led_3 = !led_state[3];
    assign Led_4 = !led_state[4];
    assign Led_5 = !led_state[5];

endmodule