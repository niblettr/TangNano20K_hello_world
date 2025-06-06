
module Top_module(
    // Clock and Reset
    input        clock,               // System Clock 27MHz (Pin4)

    // Data Ports
    inout  [7:0] DataPortPins,        // Data Port Pins (Pin73, Pin74, Pin75, Pin85, Pin77, Pin15, Pin16, Pin27)
    output reg [2:0] AddessPortPin,   // Address Port (Pin28, Pin25, Pin26)
    output reg [3:0] BOARD_X,         // Board Signals B3, B2, B1, B0 (Pin29, Pin71, Pin72, Pin29)

    // Control Signals
    output reg       TestAddressP,    // Test Address (Pin30)
    output reg       RdP,             // Read Enable (Pin31)
    output reg       WrP,             // Write Enable (Pin17)
    output reg       LampResetPin,    // Reset Signal (Pin20)
    output reg       OE_Pin,          // Level Shifter Enable (Pin19)

    // UART Signals
    output           Uart_TX_Pin,     // UART Transmit Pin (Pin55)
    input            Uart_RX_Pin,     // UART Receive Pin (Pin49)

    // Debug Signals
    output           Debug_Pin,       // Debug Toggle (Pin76)
    output           Debug_Pin2       // Debug Toggle (Pin80)
);
`include "utils.v"

    /********** Constants **********/
parameter CLOCK_FREQUENCY = 27000000;  // 27 MHz crystal oscillator
parameter BAUD_RATE = 115200;          // Uart Baud Rate (tested up to 2Mb/s - can go way higher)

parameter MAX_CMD_LENGTH = 30;
parameter CMD_LENGTH = 11; // "pb_i_write, or pb_i_read,"

 //reg TopLevelDebug2  = 0; // commented out for time being to remove warning

/*********************************************************************************************************/

/********** UART Transmission **********/
reg tx_fifo_write_en;
reg [7:0] tx_fifo_data_in = 8'b0;

/********** UART Reception **********/
reg rx_fifo_read_en;
reg [7:0] rx_fifo_data_out = 8'b0;
wire rx_fifo_empty;
wire rx_sentence_received;
reg uart_rx_previous_empty = 1'b0; // Flag to track if RX processing is in progress

/********** UART Debug String **********/
reg [7:0] uart_tx_string_index = 0; // Index for string transmission
reg [7:0] uart_tx_string [0:20];
reg [7:0] uart_tx_string_len;
reg       uart_tx_process      = 1'b0;

/********** Debug and Conversion **********/
reg [7:0] debug_hex_reg;
reg [7:0] hex_as_ascii_word [0:1] = "00";

/********** Data Ports **********/
reg data_dir; // 1 = output, 0 = input
reg  [7:0] Data_Out_Port;
wire [7:0] Data_In_Port;

/********** Command Buffers **********/
reg [7:0] command_buffer [0:32-1]; // Buffer to store the command
reg [7:0] command_param_data [0:4]; // 5 bytes (10 hex chars)

/********** Substate Machine Flags **********/
reg       lamp_card_reset_activate        = 1'b0;
reg       substate_pb_i_write4_active     = 1'b0;   // Flag to indicate if the substate machine is active
reg       substate_pb_read4_active        = 1'b0;   // Flag to indicate if the substate machine is active
reg       substate_pb_adc4_active         = 1'b0;   // Flag to indicate if the substate machine is active
reg       substate_pb_i_write4_complete   = 1'b0;   // Flag to indicate substate completion
reg       substate_pb_read4_complete      = 1'b0;   // Flag to indicate substate completion
reg       substate_pb_adc4_complete       = 1'b0;   // Flag to indicate substate completion

/********** UART State Machine **********/
reg [2:0]  uart_tx_state = 3'b000;        // State machine for UART string transmission

/********** Command Handling **********/
reg [2:0] command_state = STATE_IDLE;      // State machine for command handling
reg [5:0] command_index = 0;               // Index for the command buffer
reg [5:0] command_len   = 0;
reg [1:0] CommandType   = 0;
reg [5:0] comma_pos;

/********** Response Handling **********/
reg       ResponsePending;
reg [3:0] ResponseByteCount;
reg [7:0] ResponseBytes[0:3];

/********** Command Word **********/
wire [8*CMD_LENGTH:0] command_word = {command_buffer[0], command_buffer[1], command_buffer[2], command_buffer[3],
                                      command_buffer[4], command_buffer[5], command_buffer[6], command_buffer[7],
                                      command_buffer[8], command_buffer[9], command_buffer[10]};
/*********************************************************************************************************/

reg [7:0] PauseCount;

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
    .rx_sentence_received(rx_sentence_received)       // Connect rx_sentence_received
    //.Debug_uart(Debug_uart)
);
state_machines #(
    .CLOCK_FREQUENCY(CLOCK_FREQUENCY)
) state_machines_inst (
    .clock(clock),
    .lamp_card_reset_activate(lamp_card_reset_activate),
    .lamp_card_reset_complete(lamp_card_reset_complete),
    .substate_pb_i_write4_active(substate_pb_i_write4_active),
    .substate_pb_i_write4_complete(substate_pb_i_write4_complete), 
    .substate_pb_read4_active(substate_pb_read4_active),
    .substate_pb_read4_complete(substate_pb_read4_complete),
    .substate_pb_adc4_active(substate_pb_adc4_active),
    .substate_pb_adc4_complete(substate_pb_adc4_complete),
    .BOARD_X(BOARD_X),
    .CommandType(CommandType),
    .command_param_data(command_param_data),
    .Data_Out_Port(Data_Out_Port),
    .RdP(RdP),
    .WrP(WrP),
    .AddessPortPin(AddessPortPin),
    .TestAddressP(TestAddressP),
    .LampResetPin(LampResetPin),
    .Data_In_Port(Data_In_Port),
    .data_dir(data_dir),
    .ResponsePending(ResponsePending),
    .ResponseBytes(ResponseBytes),
    .ResponseByteCount(ResponseByteCount)
);

/*********************************************************************************************************/
task send_debug_message;
    input [7:0] debug_reg_value; // Debug register value
    input [64*8-1:0] message;    // Debug message (up to 64 characters)
    input integer message_len;   // Length of the message
    integer i;                   // Loop variable
    begin
        uart_tx_string_len <= message_len + 4; // Message length + 2 hex chars + CR + LF
        
        // Copy the message into the UART string
        for (i = 0; i < message_len; i = i + 1) begin
            uart_tx_string[i] <= message[(message_len - 1 - i) * 8 +: 8];
        end
        
        uart_tx_string[message_len]   <= hex_to_ascii_nib(debug_reg_value[7:4]);
        uart_tx_string[message_len+1] <= hex_to_ascii_nib(debug_reg_value[3:0]);
        
        uart_tx_string[message_len+2] <= 8'h0D;     // Add carriage return
        uart_tx_string[message_len+3] <= 8'h0A;     // Add line feed
        uart_tx_process <= 1'b1;                    // Trigger UART transmission
    end
endtask

/*********************************************************************************************************/



typedef enum logic [2:0] {
   STATE_INIT               = 3'b0,
   STATE_IDLE,
   STATE_PARSE_COMMAND,
   STATE_WAIT_FOR_SUBSTATE,
   STATE_PAUSE,
   STATE_PASS,
   STATE_FAIL,
   STATE_FIND_COMMA
} state_t;



always @(posedge clock) begin

    integer i;         // general purpose, remember, this is not static so will lose its value on every clock cycle.

    tx_fifo_write_en <= 1'b0;
    rx_fifo_read_en <= 1'b0;
    OE_Pin          <= 1'b1; // level shifter output enable (permanently)

    case (uart_tx_state)
        3'b000: begin
            if (uart_tx_process) begin
               uart_tx_state <= 3'b001;
            end
        end // case

        3'b001: begin
            if (uart_tx_string_index < uart_tx_string_len) begin
               tx_fifo_data_in <= uart_tx_string[uart_tx_string_index]; // Load the current character
               tx_fifo_write_en <= 1'b1;                                // Trigger UART transmission
               uart_tx_string_index <= uart_tx_string_index + 1'b1;     // Move to the next character
               uart_tx_state <= 3'b000;
             end else begin
               uart_tx_string_index <= 1'b0;     // reset index
               uart_tx_process <= 1'b0;
               uart_tx_state <= 3'b000;
             end
        end
    endcase
/*********************************************************************************************************/
    case (command_state)

        STATE_INIT: begin
            lamp_card_reset_activate <= 1;
            if(lamp_card_reset_complete) begin
               command_state <= STATE_IDLE;
               lamp_card_reset_activate <=0;

            end   
        end

        STATE_IDLE: begin // Idle state: Wait for data in RX FIFO            
            if (!rx_fifo_empty && uart_rx_previous_empty) begin
                uart_rx_previous_empty <= 1'b0;
                rx_fifo_read_en <= 1'b1; // Read from RX FIFO

                command_buffer[command_index] <= rx_fifo_data_out; // Store received byte
                command_index <= command_index + 1'b1;           
            end else if (rx_fifo_empty) begin
              uart_rx_previous_empty <= 1'b1;
            end

            if(rx_sentence_received) begin
                command_len <= command_index;
                command_index <= 3'b0;        // reset to zero
                command_state <= STATE_FIND_COMMA; // Move to command processing state
            end
        end

        STATE_FIND_COMMA: begin
           comma_pos = 0;                
           for (i = 0; i < MAX_CMD_LENGTH; i = i + 1) begin
               if (command_buffer[i] == ",") begin
                   comma_pos = i;
                   break;
               end
           end
           command_state <= STATE_PARSE_COMMAND;
        end

        STATE_PARSE_COMMAND: begin
            localparam int NUM_CMD_PARAMS = 5;

            for (i = 0; i < NUM_CMD_PARAMS; i = i + 1) begin
               logic [3:0] high_nibble, low_nibble;

               high_nibble = ascii_hex_to_nibble(command_buffer[comma_pos + 1 + i*2]);
               low_nibble  = ascii_hex_to_nibble(command_buffer[comma_pos + 2 + i*2]);

               command_param_data[i] = {high_nibble, low_nibble};
               
            end

            if (command_word == "pb_i_write,") begin
                substate_pb_i_write4_active <= 1'b1; // Activate the new state machine
                command_state <= STATE_WAIT_FOR_SUBSTATE; // Transition to a wait state
             end else if (command_word == "pb_i__read,") begin
                substate_pb_read4_active <= 1'b1; // Activate the new state machine
                command_state <= STATE_WAIT_FOR_SUBSTATE; // Transition to a wait state
             end else if (command_word == "pb_adc4_16,") begin
                CommandType = 0;
                substate_pb_adc4_active <= 1'b1; // Activate the new state machine
                command_state <= STATE_WAIT_FOR_SUBSTATE; // Transition to a wait state
             end else if (command_word == "pb_adc4_08,") begin
                CommandType = 1;
                substate_pb_adc4_active <= 1'b1; // Activate the new state machine
                command_state <= STATE_WAIT_FOR_SUBSTATE; // Transition to a wait state
            end else if (command_word == "test______,") begin
                substate_pb_adc4_active <= 1'b1; // Activate the new state machine
                command_state <= STATE_WAIT_FOR_SUBSTATE; 
            end else begin
                debug_hex_reg = 8'hFE; // example
                command_state <= STATE_FAIL;
            end
        end

        STATE_WAIT_FOR_SUBSTATE: begin // note: only one substatemachine is active at any given time...
            if (substate_pb_i_write4_complete || substate_pb_read4_complete || substate_pb_adc4_complete) begin 

                substate_pb_i_write4_active <= 1'b0; // Deactivate the substate machine
                substate_pb_read4_active    <= 1'b0; // Deactivate the substate machine
                substate_pb_adc4_active     <= 1'b0; // Deactivate the substate machine

                if(ResponsePending) begin
                   if (command_word == "pb_i__read,") begin

                      uart_tx_string[0:10] <= {"p","b","_","i","_","_","r","e","a","d",","};

                      uart_tx_string[11] <= hex_to_ascii_nib((ResponseBytes[0] & 8'hF0) >> 4);
                      uart_tx_string[12] <= hex_to_ascii_nib( ResponseBytes[0] & 8'h0F);

                      uart_tx_string[13] <= hex_to_ascii_nib((ResponseBytes[1] & 8'hF0) >> 4);
                      uart_tx_string[14] <= hex_to_ascii_nib (ResponseBytes[1] & 8'h0F);

                      uart_tx_string[15] <= hex_to_ascii_nib((ResponseBytes[2] & 8'hF0) >> 4);
                      uart_tx_string[16] <= hex_to_ascii_nib (ResponseBytes[2] & 8'h0F);

                      uart_tx_string[17] <= hex_to_ascii_nib((ResponseBytes[3] & 8'hF0) >> 4);
                      uart_tx_string[18] <= hex_to_ascii_nib (ResponseBytes[3] & 8'h0F);

                      uart_tx_string[19] <= 8'h0D;
                      uart_tx_string[20] <= 8'h0A;

                      uart_tx_string_len <= 21;
                      uart_tx_process <= 1'b1;
                   end else if (command_word == "pb_adc4_16,") begin
                      uart_tx_string[0:10] <= {"p","b","_","a","d","c","4","_","1","6",","};

                      uart_tx_string[11] <= hex_to_ascii_nib((ResponseBytes[0] & 8'hF0) >> 4);
                      uart_tx_string[12] <= hex_to_ascii_nib( ResponseBytes[0] & 8'h0F);

                      uart_tx_string[13] <= hex_to_ascii_nib((ResponseBytes[1] & 8'hF0) >> 4);
                      uart_tx_string[14] <= hex_to_ascii_nib (ResponseBytes[1] & 8'h0F);

                      uart_tx_string[15] <= hex_to_ascii_nib((ResponseBytes[2] & 8'hF0) >> 4);
                      uart_tx_string[16] <= hex_to_ascii_nib (ResponseBytes[2] & 8'h0F);

                      uart_tx_string[17] <= hex_to_ascii_nib((ResponseBytes[3] & 8'hF0) >> 4);
                      uart_tx_string[18] <= hex_to_ascii_nib (ResponseBytes[3] & 8'h0F);

                      uart_tx_string[19] <= 8'h0D;
                      uart_tx_string[20] <= 8'h0A;

                      uart_tx_string_len <= 21;
                      uart_tx_process <= 1'b1;
                   end
                end
                PauseCount <= 8'd50;
                command_state <= STATE_PAUSE;
            end

        end

        STATE_PAUSE: begin
          debug_hex_reg = 8'h56; // example
          if(PauseCount > 0) begin
             PauseCount <= PauseCount -1'b1;
          end else begin
             command_state <= STATE_PASS; // Transition to STATE_PASS
          end          
        end

        STATE_PASS: begin           
           send_debug_message(debug_hex_reg, {"P", "a", "s", "s", " ", "0", "x"}, 7);
           command_state <= STATE_IDLE;
        end

        STATE_FAIL: begin
           send_debug_message(debug_hex_reg, {"F", "a", "i", "l", "e", "d", " ", "0", "x"}, 9);
           command_state <= STATE_IDLE;
        end

        default: begin
            command_state <= STATE_IDLE; // Reset to idle state
        end
    endcase



/************************************************************************************************************************/
end

/********** Continuous Assignment **********/
assign Debug_Pin = rx_sentence_received;
assign Debug_Pin2 = clock;
//assign Debug_Pin = Debug_uart;
//assign Debug_Pin = Debug_spi;
//assign Debug_Pin = TopLevelDebug;

/********** Continuous Assignment **********/
// Assign bidirectional port with tristate buffer behavior
assign DataPortPins = (data_dir) ? Data_Out_Port : 8'bz;  // drive Data_Out_Port if output
assign Data_In_Port = DataPortPins;                      // read pins as input


endmodule