module uart_tx #(
    parameter CLOCK_FREQUENCY = 27000000,
    parameter BAUD_RATE = 115200
)(
    input clk,
    input start,
    input [7:0] data,
    output reg tx
);

    localparam BAUD_DIVISOR = CLOCK_FREQUENCY / BAUD_RATE;
    reg [15:0] baud_counter = 0;
    reg [3:0] bit_index = 0;
    reg [9:0] shift_reg = 10'b1111111111;
    reg transmitting = 1'b0;

    always @(posedge clk) begin
        if (start && !transmitting) begin
            transmitting <= 1'b1;
            shift_reg <= {1'b1, data, 1'b0}; // Start bit, data, stop bit
            bit_index <= 0;
        end

        if (transmitting) begin
            if (baud_counter < BAUD_DIVISOR - 1) begin
                baud_counter <= baud_counter + 1;
            end
            else begin
                baud_counter <= 0;
                tx <= shift_reg[0];
                shift_reg <= {1'b1, shift_reg[9:1]};
                bit_index <= bit_index + 1;

                if (bit_index == 10) begin
                    transmitting <= 1'b0;
                end
            end
        end
    end
endmodule