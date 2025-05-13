module Top_module( 
    input  clock,          // System Clock 27MHz            Pin4
    output reg [7:0] DataP,    // Data Port                 Pin73,Pin74,Pin75,Pin85,Pin77,Pin15,Pin16,Pin27
    output reg [2:0] AddessP,  // Address Port              Pin28,Pin25,Pin26
    output reg B0P,            //                           Pin29
    output reg TestAddressP,   //                           Pin30
    output reg RdP,            // Read Enable               Pin31
    output reg WrP,            // write Enable              Pin17
    output reg ResetP,         //                           Pin20
    output Uart_TX_Pin,    // Transmit pin of UART          Pin55
    input  Uart_RX_Pin,    // Receive pin of UART           Pin49
    output Debug_Pin,      // Debug toggle                  Pin76
    output Debug_clock_pin // Debug toggle                  Pin80
);

    reg TopLevelDebug  = 0;
    reg TopLevelDebug2 = 0;

    /********** Constants **********/
    parameter CLOCK_FREQUENCY = 27000000;  // 27 MHz crystal oscillator
    parameter HALF_PERIOD     = 100;       // Adjust for desired speed
    parameter integer LED_COUNT_DELAY = ((CLOCK_FREQUENCY / 1000) * HALF_PERIOD) - 1;

    parameter BAUD_RATE = 115200;          // Uart Baud Rate (tested up to 2Mb/s - can go way higher)

    /********** UART debug String **********/
    reg [3:0] uart_tx_string_index = 0; // Index for string transmission
    reg [7:0] uart_tx_string [0:63];
    reg [7:0] uart_tx_string_len;

    reg [7:0] debug_hex_reg;

    reg [7:0] debug_hex_ascii [0:1] = "AA";
/**********************************************************************/
task hex_to_ascii;
    input [7:0] hex_value;   // Input hex value
    begin
        // Append the hex value as ASCII characters
        debug_hex_ascii[0] = hex_value[7:4] < 10 ? (hex_value[7:4] + "0") : (hex_value[7:4] - 10 + "A");
        debug_hex_ascii[1] = hex_value[3:0] < 10 ? (hex_value[3:0] + "0") : (hex_value[3:0] - 10 + "A");
    end
endtask

task send_debug_message;
    input [7:0] debug_reg_value; // Debug register value
    input [64*8-1:0] message;    // Debug message (up to 64 characters)
    input integer message_len;   // Length of the message
    integer i;                   // Loop variable
    begin
        debug_hex_reg = debug_reg_value; // Set the debug register
        hex_to_ascii(debug_hex_reg);     // Convert debug register to ASCII
        uart_tx_string_len = message_len + 4; // Message length + 2 hex chars + CR + LF
        
        // Copy the message into the UART string
        for (i = 0; i < message_len; i = i + 1) begin
            uart_tx_string[i] <= message[(message_len - 1 - i) * 8 +: 8];
        end
        
        uart_tx_string[message_len] <= debug_hex_ascii[0]; // Add hex MSB
        uart_tx_string[message_len+1] <= debug_hex_ascii[1]; // Add hex LSB
        uart_tx_string[message_len+2] <= 8'h0D;     // Add carriage return
        uart_tx_string[message_len+3] <= 8'h0A;     // Add line feed
        uart_tx_process <= 1'b1;                    // Trigger UART transmission
    end
endtask
/**********************************************************************/

    /********** UART Transmission **********/
    reg tx_fifo_write_en;
    reg [7:0] tx_fifo_data_in = 8'b0;
    /********** UART Reception **********/
    reg rx_fifo_read_en;
    reg [7:0] rx_fifo_data_out = 8'b0;
    wire rx_fifo_empty;
/**************************************************************************************************************/

    uart #(
        .CLOCK_FREQUENCY(CLOCK_FREQUENCY),
        .BAUD_RATE(BAUD_RATE)
    ) uart_inst (
        .clock(clock),
        .tx_fifo_data_in(tx_fifo_data_in),
        .uart_tx_pin(Uart_TX_Pin),
        //.tx_fifo_empty(tx_fifo_empty),
        .tx_fifo_write_en(tx_fifo_write_en),

        .uart_rx_pin(Uart_RX_Pin),
        .rx_fifo_empty(rx_fifo_empty),       // Connect rx_fifo_empty
        .rx_fifo_data_out(rx_fifo_data_out), // Connect RX FIFO data output
        .rx_fifo_read_en(rx_fifo_read_en),   // Connect RX FIFO read enable

        .Debug_uart(Debug_uart)
    );

parameter CMD_LENGTH = 11; // "pb_i_write,"
reg [7:0] command_buffer [0:CMD_LENGTH-1]; // Buffer to store the command

wire [8*CMD_LENGTH:0] command_word = {command_buffer[0], command_buffer[1], command_buffer[2], command_buffer[3], command_buffer[4],
                                      command_buffer[5], command_buffer[6], command_buffer[7], command_buffer[8], command_buffer[9], command_buffer[10]};

reg [5:0] command_index = 0;               // Index for the command buffer

    // Define FSM states with meaningful names
    typedef enum logic [2:0] {
        STATE_INIT   = 3'b000,
        STATE_IDLE   = 3'b001,
        STATE_PARSE  = 3'b010,
        STATE_WAIT   = 3'b011,
        STATE_PASS   = 3'b100,
        STATE_FAIL   = 3'b101
    } state_t;

reg [2:0] command_state = STATE_INIT;      // State machine for command handling

reg uart_rx_previous_empty = 1'b0; // Flag to track if RX processing is in progress
reg uart_tx_process        = 1'b0;

reg [2:0] uart_tx_state = 3'b000; // State machine for UART string transmission
reg [7:0] reset_counter = 8'b0; // 1-bit counter for reset delay
always @(posedge clock) begin

    rx_fifo_read_en <= 1'b0;

    case (uart_tx_state)
        3'b000: begin
            if (uart_tx_process) begin
               uart_tx_state <= 3'b001;
            end
        end // case

        3'b001: begin
            if (uart_tx_string_index < uart_tx_string_len) begin
               tx_fifo_data_in <= uart_tx_string[uart_tx_string_index]; // Load the current character
               tx_fifo_write_en <= 1'b1;                          // Trigger UART transmission
               uart_tx_string_index <= uart_tx_string_index + 1'b1;     // Move to the next character
               uart_tx_state <= 3'b011;
             end else begin
               uart_tx_string_index <= 1'b0;     // reset index
               uart_tx_process <= 1'b0;
               uart_tx_state <= 3'b000;
             end
        end
        
        3'b011: begin
           tx_fifo_write_en <= 1'b0;         // Deassert start signal one clock cycle later
           uart_tx_state <= 3'b001;          // move back to start
        end

    endcase
/*********************************************************************************************************/
    case (command_state)
        STATE_INIT: begin // we want to reset fifo here then move on..
           if(reset_counter < 100) begin
              reset_counter <= reset_counter + 1'b1;
              ResetP       <= 1'b1;        // hold in reset
              DataP        <= 8'b0;
              AddessP      <= 3'b0;
              TestAddressP <= 1'b0;
              B0P          <= 1'b0;
              RdP          <= 1'b0;
              WrP          <= 1'b0;
           end else begin
              ResetP       <= 1'b0;   // release from reset state
              B0P          <= 1'b1;   // enable the board
              command_state <= STATE_IDLE;
           end
        end

        STATE_IDLE: begin
            // Idle state: Wait for data in RX FIFO
            if (!rx_fifo_empty && uart_rx_previous_empty) begin
                uart_rx_previous_empty <= 1'b0;
                rx_fifo_read_en <= 1'b1; // Read from RX FIFO

                command_buffer[command_index] <= rx_fifo_data_out; // Store received byte
                if (command_index == CMD_LENGTH -1 ) begin
                    command_index <= 3'b0;        // reset to zero
                    command_state <= STATE_PARSE; // Move to command processing state
                end else begin
                   command_index <= command_index + 1'b1; // Increment buffer index
                end
            end else if (rx_fifo_empty) begin
              uart_rx_previous_empty <= 1'b1;
            end
        end

        STATE_PARSE: begin
            if (command_word == "pb_i_write,") begin

               command_state <= STATE_PASS;
            end else begin
              command_state <= STATE_FAIL;
            end
        end // case

        STATE_PASS: begin
           debug_hex_reg = 8'h55; // example
           send_debug_message(debug_hex_reg, {"P", "a", "s", "s", " ", "0", "x"}, 7);
           command_state <= STATE_IDLE;
        end

        STATE_FAIL: begin
           debug_hex_reg = 8'hAA; // example
           send_debug_message(debug_hex_reg, {"F", "a", "i", "l", "e", "d", " ", "0", "x"}, 9);
           command_state <= STATE_IDLE;
        end

        default: begin
            command_state <= STATE_IDLE; // Reset to idle state
        end
    endcase
end

/*
reg uart_rx_processing = 1'b0; // Flag to track if RX processing is in progress
always @(posedge clock) begin
    // Default assignments
    rx_fifo_read_en  <= 1'b0;         // Deassert UART RX FIFO read enable
    tx_fifo_write_en <= 1'b0;         // Deassert UART TX FIFO write enable

    // Echo Uart RX data back out the Uart TX using the fifo
    if (!rx_fifo_empty && !uart_rx_processing) begin  // !rx_fifo_read_en gets rid of weird echo...FIX ASAP!!!
        rx_fifo_read_en  <= 1'b1;                  // Assert read enable to read from RX FIFO
        tx_fifo_write_en <= 1'b1;
        uart_rx_processing <= 1'b1;                  // Set processing flag
        tx_fifo_data_in <= rx_fifo_data_out;       // Load RX FIFO data into UART TX
    end else if (rx_fifo_empty) begin
        uart_rx_processing <= 1'b0;                  // Clear processing flag when FIFO is empty
    end
end
*/

/********** Continuous Assignment **********/
//assign Debug_clock_pin = TopLevelDebug2;
//assign Debug_Pin = Debug_uart;
//assign Debug_Pin = Debug_spi;
assign Debug_Pin = TopLevelDebug;

endmodule