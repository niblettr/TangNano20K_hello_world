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

    /********** Counter **********/
    // parameter Clock_frequency = 27_000_000; // Crystal oscillator frequency is 27MHz
    parameter count_value_05S = 13_499_999; // The number of clock cycles needed to time 0.5 seconds
    parameter count_value_01S = 2_699_999;  // The number of clock cycles needed to time 0.1 seconds

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

/*niblett NOTES:
In Verilog, the <= operator is used for non-blocking assignments within an always block. Non-blocking 
assignments are typically used in sequential logic (e.g., flip-flops) to ensure that all right-hand 
side expressions are evaluated before any left-hand side assignments are made. This helps to model the 
behavior of hardware more accurately, especially when dealing with clocked processes.

Here's an example from your code:
always @(posedge Clock) begin
    if ( count_value_reg <= count_value ) begin //not count to 0.5S
        count_value_reg  <= count_value_reg + 1'b1; // Continue counting
        count_value_flag <= 1'b0 ; // No flip flag
    end
    else begin //Count to 0.5S
        count_value_reg  <= 23'b0; // Clear counter,prepare for next time counting.
        count_value_flag <= 1'b1 ; // Flip flag
    end
end

In this always block, the <= operator is used to assign new values to count_value_reg and count_value_flag 
on the positive edge of the Clock. The non-blocking assignment ensures that all assignments within the block 
are updated simultaneously at the end of the current simulation time step, which is crucial for correctly 
modeling synchronous digital circuits.

*/