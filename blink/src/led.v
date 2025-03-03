/*
Module Declaration:
module led: Declares a module named led.
input Clock: Declares an input port named Clock.
output IO_voltage: Declares an output port named IO_voltage.
This module is designed to control the output voltage (IO_voltage) based on the input clock signal (Clock).
*/

// NOTE the use of Non-Blocking Assignment (<=), so that eveything udated in parallel.

module led( 
    input  Clock,
    output IO_voltage,
    output IO_voltage2,
    output IO_voltage3
);

    /********** Constants **********/
    parameter  CLOCK_FREQUENCY = 27000000;  // Crystal oscillator frequency is 27MHz
    parameter  HALF_PERIOD_1   = 500;
    parameter  HALF_PERIOD_2   = 100;

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
    reg IO_voltage_reg  = 1'b0; // Initial state
    reg IO_voltage_reg2 = 1'b0; // Initial state

    //IO_voltage_reg  <= IO_voltage_reg  ^ (count_value_reg  == 0);  // results in 2ns skew DON'T TRY TO BE CLEVER
    //IO_voltage_reg2 <= IO_voltage_reg2 ^ (count_value_reg2 == 0);  // results in 2ns skew DON'T TRY TO BE CLEVER
        
    always @(posedge Clock) begin
        IO_voltage_reg  <= (count_value_reg  == 0) ? ~IO_voltage_reg  : IO_voltage_reg;
        IO_voltage_reg2 <= (count_value_reg2 == 0) ? ~IO_voltage_reg2 : IO_voltage_reg2;
      
    end

    /* Continuous Assignments: links shit to the outside world, i.e. pins */
    assign IO_voltage  = IO_voltage_reg;
    assign IO_voltage2 = IO_voltage_reg2;

    assign IO_voltage3 = Clock;  // Toggle on every clock cycle (27MHZ)

endmodule

