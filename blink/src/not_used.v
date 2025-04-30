/****************************************************************************************************/
// test statemachine just for printing "Test" string out the uart
/*
reg [2:0] uart_tx_state = 3'b000; // State machine for UART string transmission
reg [32:0] wait_delay = 32'b0;
always @(posedge clock) begin

    case (uart_tx_state)
        3'b000: begin
            if (uart_string_index < uart_string_len) begin
                    tx_fifo_data_in <= uart_string[uart_string_index]; // Load the current character
                    tx_fifo_write_en <= 1'b1;                          // Trigger UART transmission
                    uart_string_index <= uart_string_index + 1'b1;     // Move to the next character
              uart_tx_state <= 3'b010;
                end else begin
              uart_string_index <= 1'b0;     // reset index
              uart_tx_state <= 3'b011;       // Move to the wait state
                end
            end
        3'b010: begin
           tx_fifo_write_en <= 1'b0;         // Deassert start signal one clock cycle later
           uart_tx_state <= 3'b000;          // move back to start
        end

        3'b011: begin
            tx_fifo_write_en <= 1'b0;            // Deassert start signal
            if(wait_delay == 32'd2700000) begin  // 100ms
                wait_delay <= 32'b0;             // Reset wait delay
                uart_tx_state <= 3'b000;         // Go back to the first state
            end else begin
                wait_delay <= wait_delay + 1'b1; // Increment wait delay
            end
        end
    endcase
end

*/
/*****************************************************************************************/
/*
reg uart_rx_processing = 1'b0; // Flag to track if RX processing is in progress
always @(posedge clock) begin
    // Default assignments
    rx_fifo_read_en  <= 1'b0;         // Deassert UART RX FIFO read enable
    tx_fifo_write_en <= 1'b0;         // Deassert UART TX FIFO write enable

    // Echo Uart RX data back out the Uart TX using the fifo
    if (!rx_fifo_empty && !uart_rx_processing) begin  // !rx_fifo_read_en gets rid of weird echo...FIX ASAP!!!
        //TopLevelDebug <= ~TopLevelDebug;
        rx_fifo_read_en  <= 1'b1;                  // Assert read enable to read from RX FIFO
        tx_fifo_write_en <= 1'b1;
        uart_rx_processing <= 1'b1;                  // Set processing flag
        tx_fifo_data_in <= rx_fifo_data_out;       // Load RX FIFO data into UART TX
    end else if (rx_fifo_empty) begin
        uart_rx_processing <= 1'b0;                  // Clear processing flag when FIFO is empty
    end
end
*/