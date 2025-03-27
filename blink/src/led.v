/*
Module Declaration:
module led: Declares a module named led.
input Clock: Declares an input port named Clock.
output Led_0: Declares an output port named Led_0.
This module is designed to control the output voltage (Led_0) based on the input clock signal (Clock).
*/

// NOTE the use of Non-Blocking Assignment (<=), so that eveything udated in parallel.

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
    parameter  HALF_PERIOD_1   = 500;
    parameter  HALF_PERIOD_2   = 500;

    parameter  integer  COUNT_05S = ( ( CLOCK_FREQUENCY / 1000) * HALF_PERIOD_1 ) - 1;
    parameter  integer  COUNT_01S = ( ( CLOCK_FREQUENCY / 1000) * HALF_PERIOD_2 ) - 1;


    /********** Counters **********/
    reg [23:0] count_value_reg  = 0; // Counter register (24 bits)
    reg [23:0] count_value_reg2 = 0; // Counter register (24 bits)

    /****************************************************************************************************/
    always @(posedge Clock) begin
        // Counter for 0.5 seconds
        if (count_value_reg < COUNT_05S) begin
            count_value_reg <= count_value_reg + 1; // Increment counter
        end
        else begin
            count_value_reg <= 0; // Reset counter
        end

        // Counter for 0.1 seconds
        if (count_value_reg2 < COUNT_01S) begin
            count_value_reg2 <= count_value_reg2 + 1; // Increment counter
        end
        else begin
            count_value_reg2 <= 0; // Reset counter
        end
    end

    /********** IO voltage flip **********/
    reg Led_0_Reg = 1'b0; // Initial state
    reg Led_5_Reg = 1'b0; // Initial state

    /****************************************************************************************************/
       
    always @(posedge Clock) begin
        Led_0_Reg  <= (count_value_reg  == 0) ? ~Led_0_Reg  : Led_0_Reg;
        Led_5_Reg <= (count_value_reg2 == 0) ? ~Led_5_Reg : Led_5_Reg;
    end

    /* Continuous Assignments: links shit to the outside world, i.e. pins */
    assign Led_0 = Led_0_Reg;
    assign Led_5 = Led_5_Reg;

endmodule

