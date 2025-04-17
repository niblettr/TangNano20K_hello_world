module led_scroll #(
    parameter CLOCK_FREQUENCY = 27000000, // System clock frequency in Hz
    parameter LED_COUNT_DELAY     = 1000 // some crap default 
)(
    input        clock,     // System clock
    output [5:0] leds
);


    reg [23:0] count_value_reg = 0; // Counter for LED delay
    reg [31:0] uart_counter = 0;    // Counter for 1-second UART delay

    /********** LED State (Shift Register) **********/
    reg [5:0] led_state = 6'b000001; // Initial state: Only Led_0 is ON
    reg direction = 1'b1;           // 1 for right, 0 for left

    /****************************************************************************************************/
    // LED handling
    
    always @(posedge clock) begin
        // Counter for delay
        if (count_value_reg < LED_COUNT_DELAY) begin
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


/********** Continuous Assignments **********/
    generate
        genvar i; // Declare a generate variable 'i'
        for (i = 0; i < 6; i = i + 1) begin
            assign leds[i] = ~led_state[i]; // Invert because active low
        end
    endgenerate

endmodule