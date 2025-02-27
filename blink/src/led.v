/*
Module Declaration:
module led: Declares a module named led.
input Clock: Declares an input port named Clock.
output IO_voltage: Declares an output port named IO_voltage.
This module is designed to control the output voltage (IO_voltage) based on the input clock signal (Clock).
*/

module led( 
    input  Clock,
    output IO_voltage,
    output IO_voltage2
);

    /********** Constants **********/
    parameter  CLOCK_FREQUENCY = 27_000_000;  // Crystal oscillator frequency is 27MHz
    parameter  integer  COUNT_05S = ((CLOCK_FREQUENCY * 500) / 1_000) - 1; // 13_499_999 count = 500ms
    parameter  integer  COUNT_01S = ((CLOCK_FREQUENCY * 100) / 1_000) - 1; // 2_699_999 count = 100ms


    /********** Counters **********/
    // the following does not work properly 
    parameter count_value_05S = COUNT_05S;  // The number of clock cycles needed to time 0.5 seconds
    parameter count_value_01S = COUNT_01S;  // The number of clock cycles needed to time 0.1 seconds

    // this works fine
    //parameter count_value_05S = 13_499_999;  // The number of clock cycles needed to time 0.5 seconds
    //parameter count_value_01S = 2_699_999;  // The number of clock cycles needed to time 0.1 seconds

    // the following does not even compile!!!!
    //$display("count_value_05S = %d", count_value_05S);

    reg [23:0] count_value_reg  = 24'b0; // Counter register (24 bits)
    reg [23:0] count_value_reg2 = 24'b0; // Counter register (24 bits)

    /****************************************************************************************************/
    always @(posedge Clock) begin
        // Counter for 0.5 seconds
        if (count_value_reg < count_value_05S) begin
            count_value_reg <= count_value_reg + 1; // Increment counter
        end
        else begin
            count_value_reg <= 24'b0; // Reset counter
        end

        // Counter for 0.1 seconds
        if (count_value_reg2 < count_value_01S) begin
            count_value_reg2 <= count_value_reg2 + 1; // Increment counter
        end
        else begin
            count_value_reg2 <= 24'b0; // Reset counter
        end
    end

    /********** IO voltage flip **********/
    reg IO_voltage_reg  = 1'b0; // Initial state
    reg IO_voltage_reg2 = 1'b0; // Initial state

    always @(posedge Clock) begin
        /****************************************************************************************************/
        IO_voltage_reg  <= IO_voltage_reg  ^ (count_value_reg  == 0);
        IO_voltage_reg2 <= IO_voltage_reg2 ^ (count_value_reg2 == 0);
        /****************************************************************************************************/
    end

    /* Continuous Assignment: continuously drives the value of IO_voltage with the value of IO_voltage_reg */
    assign IO_voltage  = IO_voltage_reg;
    assign IO_voltage2 = IO_voltage_reg2;

endmodule

