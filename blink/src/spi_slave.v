module spi_slave (
    input  wire       system_clk,     // System clock
    input  wire       spi_clk,        // SPI clock (from master)
    input  wire       spi_cs,         // Chip select (active low)
    input  wire       mosi,           // Master Out, Slave In
    output wire       miso,           // Master In, Slave Out
    output reg        data_ready,     // Data received flag
    input  wire       read_ack,       // Acknowledgment from master to clear flag
    output reg [7:0]  received_data,  // Received byte
    input  wire [7:0]  data_to_send,   // Data to send back
    output reg [7:0]  Debug           // Debug counter for rising edges
);

    // Internal registers
    reg [7:0] shift_reg = 8'b0;       // Shift register for receiving data
    reg [2:0] bit_count = 3'b0;       // Bit counter
    reg [7:0] miso_reg = 8'b0;        // Register to hold data for transmission

    // Synchronize SPI clock into system clock domain
    reg spi_clk_sync1, spi_clk_sync2, spi_clk_sync3;

    always @(posedge system_clk) begin
        spi_clk_sync1 <= spi_clk;
        spi_clk_sync2 <= spi_clk_sync1;
        spi_clk_sync3 <= spi_clk_sync2;
    end

    wire spi_rising_edge  = (spi_clk_sync3 == 1'b0) && (spi_clk_sync2 == 1'b1);
    wire spi_falling_edge = (spi_clk_sync3 == 1'b1) && (spi_clk_sync2 == 1'b0);

    // SPI communication logic
    always @(posedge system_clk) begin
        if (spi_cs == 1'b1) begin
            // SPI not active — reset internal state
            bit_count   <= 3'b0;
            data_ready  <= 1'b0;
            miso_reg    <= data_to_send; // Load full byte to transmit
        end else begin
            if (spi_rising_edge) begin
                shift_reg <= {shift_reg[6:0], mosi}; // Shift in data
                bit_count <= bit_count + 1'b1;
                Debug <= Debug + 1'b1;

                if (bit_count == 3'b111) begin
                    received_data <= {shift_reg[6:0], mosi}; // Latch full byte
                    data_ready <= 1'b1;
                end
            end

            if (spi_falling_edge) begin
                // Shift out data from MSB to LSB
                miso_reg <= {miso_reg[6:0], 1'b0}; // Shift left by one
            end
        end

        // Clear data_ready when acknowledged
        if (read_ack) begin
            data_ready <= 1'b0;
        end
    end

    // Connect miso to MSB of shift register (tri-state control if needed)
    assign miso = (spi_cs == 1'b0) ? miso_reg[7] : 1'bz;

endmodule