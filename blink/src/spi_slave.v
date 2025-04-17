module spi_slave (
    input  wire       clock,     // System clock
    input  wire       spi_clk,        // SPI clock (from master)
    input  wire       spi_cs,         // Chip select (active low)
    input  wire       mosi,           // Master Out, Slave In
    output wire       miso,           // Master In, Slave Out
    output reg        spi_data_ready, // Data received flag
    input  wire       spi_read_ack,   // Acknowledgment from master to clear flag
    output reg [7:0]  spi_rx_data,    // Received spi byte
    input  wire [7:0] data_to_send,   // Data to send back
    output reg        Debug_spi       // routed to pin in top module, for debug purposes...
);

    // Internal registers
    reg [7:0] shift_reg = 8'b0;       // Shift register for receiving data
    reg [2:0] bit_count = 3'b0;       // Bit counter
    reg [7:0] miso_reg = 8'b0;        // Register to hold data for transmission

    // Synchronize SPI clock into system clock domain
    reg spi_clk_sync1, spi_clk_sync2, spi_clk_sync3;

    always @(posedge clock) begin
        spi_clk_sync1 <= spi_clk;
        spi_clk_sync2 <= spi_clk_sync1;
        spi_clk_sync3 <= spi_clk_sync2;
    end

    wire spi_rising_edge  = (spi_clk_sync3 == 1'b0) && (spi_clk_sync2 == 1'b1);
    wire spi_falling_edge = (spi_clk_sync3 == 1'b1) && (spi_clk_sync2 == 1'b0);

    // SPI communication logic
    always @(posedge clock) begin
        if (spi_cs == 1'b1) begin
            // SPI not active — reset internal state
            bit_count       <= 3'b0;
            spi_data_ready  <= 1'b0;
            miso_reg        <= data_to_send; // Load full byte to transmit
        end else begin
            if (spi_rising_edge) begin
                shift_reg <= {shift_reg[6:0], mosi}; // Shift in data
                bit_count <= bit_count + 1'b1;
                //Debug_spi = ~Debug_spi;

                if (bit_count == 3'b111) begin
                    spi_rx_data    <= {shift_reg[6:0], mosi}; // Latch full byte
                    spi_data_ready <= 1'b1;
                    //Debug_spi = ~Debug_spi;
                end
            end

            if (spi_falling_edge) begin
                // Shift out data from MSB to LSB
                miso_reg <= {miso_reg[6:0], 1'b0}; // Shift left by one
            end
        end

        // Clear spi_data_ready when acknowledged
        if (spi_read_ack) begin
            spi_data_ready <= 1'b0;
        end
    end

    // Connect miso to MSB of shift register (tri-state control if needed)
    assign miso = (spi_cs == 1'b0) ? miso_reg[7] : 1'bz;

endmodule