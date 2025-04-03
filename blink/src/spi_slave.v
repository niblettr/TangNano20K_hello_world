module spi_slave (
    input  wire       system_clk,    // System clock
    input  wire       spi_clk,       // SPI clock
    input  wire       spi_cs,        // Chip select (active low)
    input  wire       mosi,          // Master Out, Slave In
    output reg        miso,          // Master In, Slave Out
    output reg        data_ready,    // Data received flag
    input  wire       read_ack,      // Acknowledgment from master to clear flag
    output reg [7:0]  received_data, // Received byte
    input  wire [7:0] data_to_send,  // Data to send back
    output reg        Debug
);

    // Internal registers
    reg [7:0] shift_reg = 8'b0;      // Shift register for receiving data
    reg [2:0] bit_count = 3'b0;      // Bit counter for SPI byte
    reg sck_prev = 1'b0;             // Previous state of SPI clock (spi_clk)


    reg spi_clk_sync1, spi_clk_sync2;
    always @(posedge system_clk) begin
        spi_clk_sync1 <= spi_clk;
        spi_clk_sync2 <= spi_clk_sync1;
    end
    wire spi_clk_stable = spi_clk_sync2;



    always @(posedge system_clk) begin
        // Track the previous state of SCK
        sck_prev <= spi_clk_stable;
        //miso <= spi_clk;

        // Reset logic when CS is high
        if (spi_cs == 1'b1) begin
                  
            bit_count <= 3'b0;       // Reset bit counter
            data_ready <= 1'b0;     // Clear data ready flag
            miso <= data_to_send[7]; // Prepare first bit to send
        end else begin
            // Rising edge of SCK: Shift in data from MOSI
            if ((sck_prev == 1'b0) && (spi_clk_stable == 1'b1)) begin
                shift_reg <= {shift_reg[6:0], mosi}; // Shift in data
                bit_count <= bit_count + 1'b1;
                Debug <= Debug + 1'b1;

                // If a full byte is received
                if (bit_count == 3'b111) begin
                    received_data <= {shift_reg[6:0], mosi}; // Latch received byte
                    data_ready <= 1'b1;                     // Indicate data is ready
                end
            end

            // Falling edge of SCK: Shift out data to MISO
            if ((sck_prev == 1'b1) && (spi_clk_stable == 1'b0)) begin
                miso <= data_to_send[7 - bit_count]; // Shift out next bit
            end
        end

        // Clear the data_ready flag when read_ack is asserted
        if (read_ack) begin
            data_ready <= 1'b0;
        end
    end
endmodule