module spi_slave (
    input  wire       clk,       // System clock
    input  wire       sck,       // SPI clock
    input  wire       cs,        // Chip select (active low)
    input  wire       mosi,      // Master Out, Slave In
    output reg        miso,      // Master In, Slave Out
    output reg        data_ready,// Data received flag
    input  wire       read_ack,  // Acknowledgment from master to clear flag
    output reg [7:0]  received_data, // Received byte
    input  wire [7:0] data_to_send   // Data to send back
);

    reg [7:0] shift_reg = 8'b0;
    reg [2:0] bit_count = 3'b0;
    reg sck_prev = 1'b0;
    reg cs_prev = 1'b1;

    always @(posedge clk) begin
        sck_prev <= sck;
        cs_prev  <= cs;
        
        if (cs == 1'b1) begin // Reset when CS is high
            bit_count <= 3'b0;
            data_ready <= 1'b0;
            miso <= data_to_send[7]; // Prepare first bit
        end
        else if (cs == 1'b0 && sck_prev == 1'b0 && sck == 1'b1) begin // Rising edge of SCK
            shift_reg <= {shift_reg[6:0], mosi}; // Shift in data
            bit_count <= bit_count + 1;
            
            if (bit_count == 3'b111) begin // Byte received
                received_data <= {shift_reg[6:0], mosi};
                data_ready <= 1'b1;
            end
        end
        else if (cs == 1'b0 && sck_prev == 1'b1 && sck == 1'b0) begin // Falling edge of SCK
            miso <= data_to_send[7 - bit_count]; // Shift out next bit
        end

        if (read_ack) begin // Clear data ready flag when master acknowledges
            data_ready <= 1'b0;
        end
    end
endmodule