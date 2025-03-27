module led( 
    input  Clock,
    output Led_0, //Pin15
    output Led_1, //Pin16
    output Led_2, //Pin17
    output Led_3, //Pin18
    output Led_4, //Pin19
    output Led_5  //Pin20
);

    /********** Constants **********/
    parameter  CLOCK_FREQUENCY = 27000000;  // Crystal oscillator frequency is 27MHz
    parameter  HALF_PERIOD     = 100;       // Adjust for desired speed

    parameter  integer  COUNT_DELAY = ( ( CLOCK_FREQUENCY / 1000) * HALF_PERIOD ) - 1;

    /********** Counters **********/
    reg [23:0] count_value_reg = 0; // Counter register (24 bits)

    /********** LED State **********/
    reg [5:0] led_state = 6'b000001; // Initial state: Only Led_0 is ON
    reg direction = 1'b1;           // 1 for right, 0 for left

    /****************************************************************************************************/
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

    /********** Continuous Assignments **********/
    assign Led_0 = !led_state[0];
    assign Led_1 = !led_state[1];
    assign Led_2 = !led_state[2];
    assign Led_3 = !led_state[3];
    assign Led_4 = !led_state[4];
    assign Led_5 = !led_state[5];

endmodule