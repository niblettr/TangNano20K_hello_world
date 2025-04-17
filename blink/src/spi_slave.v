module spi_slave (
    input  wire       clock,     // System clock
    input  wire       spi_clk,        // SPI clock (from master)
    input  wire       spi_cs,         // Chip select (active low)
    input  wire       mosi,           // Master Out, Slave In
    output wire       miso,           // Master In, Slave Out
    input  wire [7:0] data_to_send,      // Data to send back, NOT USED ATM

    output reg  [7:0] spi_fifo_data_out,
    input  wire       spi_fifo_read_en,
    output reg        spi_fifo_empty,

    output reg        Debug_spi       // routed to pin in top module, for debug purposes...
);

    // Internal registers
    reg [7:0] shift_reg = 8'b0;       // Shift register for receiving data
    reg [2:0] bit_count = 3'b0;       // Bit counter
    reg [7:0] miso_reg = 8'b0;        // Register to hold data for transmission

    reg spi_fifo_reset         = 1'b0;
    reg spi_fifo_write_en      = 1'b0;
    reg [7:0] spi_fifo_data_in = 8'b0;

    fifo #(
        .DATA_WIDTH(8),
        .DEPTH(64)
    ) fifo_spi_slave_inst (
        .clock(clock),
        .reset(spi_fifo_reset),
        .write_en(spi_fifo_write_en),
        .read_en(spi_fifo_read_en),
        .data_in(spi_fifo_data_in),
        .data_out(spi_fifo_data_out),
        .full(spi_fifo_full),
        .empty(spi_fifo_empty),
        .Debug_fifo(Debug_spi_dummy)
    );


    // Initialize uart_tx_pin to idle state (high) and FIFO ready flag
    reg [3:0] reset_counter = 4'b0; // 4-bit counter for reset delay
    reg reset_done = 1'b0;          // Flag to indicate reset has been deasserted

    //reg Debug_uart_dummy = 1'b0;    // eventually map to another debug pin... Does nothing ATM..

    always @(posedge clock) begin
        if (!reset_done) begin
            if (reset_counter < 4'd10) begin
                reset_counter <= reset_counter + 1'b1;
                spi_fifo_reset <= 1'b1;
            end else begin
                reset_done <= 1'b1;    // Latch that reset is done
                spi_fifo_reset <= 1'b0;
            end
        end
    end


    // Synchronize SPI clock into system clock domain
    reg spi_clk_sync1, spi_clk_sync2, spi_clk_sync3;

    always @(posedge clock) begin
        spi_clk_sync1 <= spi_clk;
        spi_clk_sync2 <= spi_clk_sync1;
        spi_clk_sync3 <= spi_clk_sync2;
    end

    wire spi_rising_edge  = (spi_clk_sync3 == 1'b0) && (spi_clk_sync2 == 1'b1);
    wire spi_falling_edge = (spi_clk_sync3 == 1'b1) && (spi_clk_sync2 == 1'b0);

    // SPI Receive logic
    always @(posedge clock) begin
       spi_fifo_write_en <= 1'b0;             // Deassert write enable WORKS!!!!!!!!!!!!!!!!!!!!! LINE 74
       if (spi_cs == 1'b1) begin              // Reset logic when chip select is deasserted        
          bit_count <= 3'b0;                  // Reset bit counter
          miso_reg  <= data_to_send;          // Load full byte to transmit
       end else begin
          if (spi_rising_edge) begin
             // Shift in data on SPI clock rising edge
             shift_reg <= {shift_reg[6:0], mosi}; // Shift in data from MOSI
             bit_count <= bit_count + 1'b1;       // Increment bit counter
             if (bit_count == 3'b111) begin
                // When a full byte is received
                spi_fifo_write_en <= 1'b1;                  // Enable write to FIFO
                spi_fifo_data_in <= {shift_reg[6:0], mosi}; // Latch full byte into FIFO
                Debug_spi <= ~Debug_spi;                    // Toggle debug signal
             end else begin
                spi_fifo_write_en <= 1'b0;                  // Deassert write enable DOES NOT WORK LINE 90
             end
          end
          if (spi_falling_edge) begin
             // Shift out data on SPI clock falling edge
             miso_reg <= {miso_reg[6:0], 1'b0}; // Shift out data from MSB to LSB
          end
       end
    end
    // Connect miso to MSB of shift register (tri-state control if needed)
    assign miso = (spi_cs == 1'b0) ? miso_reg[7] : 1'bz;
endmodule