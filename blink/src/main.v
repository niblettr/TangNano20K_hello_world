
module Top_module( 
    input  clock,                     // System Clock 27MHz   Pin4

    inout [7:0] DataPortPins,         //                      Pin73,Pin74,Pin75,Pin85,Pin77,Pin15,Pin16,Pin27
    output reg [2:0] AddessPortPin,   // Address Port         Pin28,Pin25,Pin26
    output reg [3:0] BOARD_X,         // B3,B2,B1,B0          Pin29, Pin71, Pin72, Pin29
    output reg TestAddressP,          // Test address         Pin30
    output reg RdP,                   // Read Enable          Pin31
    output reg WrP,                   // write Enable         Pin17
    output reg LampResetPin,          // reset                Pin20
    output reg OE_Pin,                // LevelShifter Enable  Pin19

    output Uart_TX_Pin,               // Transmit pin of UART Pin55
    input  Uart_RX_Pin,               // Receive pin of UART  Pin49
    output Debug_Pin,                 // Debug toggle         Pin76
    output Debug_Pin2                 // Debug toggle         Pin80
);

`include "utils.v"

    /********** Constants **********/
    parameter CLOCK_FREQUENCY = 27000000;  // 27 MHz crystal oscillator
    parameter HALF_PERIOD     = 100;       // Adjust for desired speed
    parameter integer LED_COUNT_DELAY = ((CLOCK_FREQUENCY / 1000) * HALF_PERIOD) - 1;

    parameter BAUD_RATE = 115200;          // Uart Baud Rate (tested up to 2Mb/s - can go way higher)

 //reg TopLevelDebug2  = 0; // commented out for time being to remove warning
/*********************************************************************************************************/





/*********************************************************************************************************/

/********** UART Transmission **********/
reg tx_fifo_write_en;
reg [7:0] tx_fifo_data_in = 8'b0;
/********** UART Reception **********/
reg rx_fifo_read_en;
reg [7:0] rx_fifo_data_out = 8'b0;
wire rx_fifo_empty;
wire rx_sentence_received;


    /********** UART debug String **********/
reg [7:0] uart_tx_string_index = 0; // Index for string transmission
reg [7:0] uart_tx_string [0:20];
reg [7:0] uart_tx_string_len;

reg [7:0] debug_hex_reg;
reg [7:0] hex_as_ascii_word [0:1] = "00";


reg [7:0]  Data_Out_Port;
wire [7:0] Data_In_Port;


parameter MAX_CMD_LENGTH = 30;
parameter CMD_LENGTH = 11; // "pb_i_write, or pb_i_read,"
reg [7:0] command_buffer [0:32-1]; // Buffer to store the command


integer comma_pos;
integer i;         // general purpose
reg [7:0] command_param_data [0:4]; // 5 bytes (10 hex chars)

reg lamp_card_reset_activate        = 1'b0;
reg substate_pb_i_write4_active     = 1'b0;   // Flag to indicate if the substate machine is active
reg substate_pb_read4_active        = 1'b0;   // Flag to indicate if the substate machine is active
reg substate_pb_adc4_active         = 1'b0;   // Flag to indicate if the substate machine is active
reg substate_pb_i_write4_complete   = 1'b0;   // Flag to indicate substate completion
reg substate_pb_read4_complete      = 1'b0;   // Flag to indicate substate completion
reg substate_pb_adc4_complete       = 1'b0;   // Flag to indicate substate completion

reg [2:0] uart_tx_state    = 3'b000; // State machine for UART string transmission

reg uart_tx_process        = 1'b0;
reg uart_rx_previous_empty = 1'b0; // Flag to track if RX processing is in progress

reg [2:0] command_state = STATE_IDLE;      // State machine for command handling
reg [5:0] command_index = 0;               // Index for the command buffer
reg [5:0] command_len = 0;
reg [1:0] CommandType            = 0;

reg data_dir; // 1 = output, 0 = input

wire [8*CMD_LENGTH:0] command_word = {command_buffer[0], command_buffer[1], command_buffer[2], command_buffer[3],
                                      command_buffer[4], command_buffer[5], command_buffer[6], command_buffer[7],
                                      command_buffer[8], command_buffer[9], command_buffer[10]};

wire [8*CMD_LENGTH:0] command_data_debug = {command_buffer[11], command_buffer[12], command_buffer[13], command_buffer[14],
                                            command_buffer[15]};
/*********************************************************************************************************/



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
        .command_param_data(command_param_data),
        .Data_Out_Port(Data_Out_Port),
        .RdP(RdP),
        .WrP(WrP),
        .AddessPortPin(AddessPortPin),
        .TestAddressP(TestAddressP),
        .LampResetPin(LampResetPin),
        .Data_In_Port(Data_In_Port),
        .data_dir(data_dir)

        //.debug_hex_reg(debug_hex_reg)

    );

/*********************************************************************************************************/
task send_debug_message;
    input [7:0] debug_reg_value; // Debug register value
    input [64*8-1:0] message;    // Debug message (up to 64 characters)
    input integer message_len;   // Length of the message
    integer i;                   // Loop variable
    begin
      // debug_hex_reg = debug_reg_value; // Set the debug register
       // hex_to_ascii_task(debug_hex_reg);     // Convert debug register to ASCII
        uart_tx_string_len <= message_len + 4; // Message length + 2 hex chars + CR + LF
        
        // Copy the message into the UART string
        for (i = 0; i < message_len; i = i + 1) begin
            uart_tx_string[i] <= message[(message_len - 1 - i) * 8 +: 8];
        end
        
        //uart_tx_string[message_len]   <= hex_as_ascii_word[0]; // Add hex MSB
        //uart_tx_string[message_len+1] <= hex_as_ascii_word[1]; // Add hex LSB
        uart_tx_string[message_len]   <= hex_to_ascii_nib(debug_reg_value[7:4]);
        uart_tx_string[message_len+1] <= hex_to_ascii_nib(debug_reg_value[3:0]);
        
        uart_tx_string[message_len+2] <= 8'h0D;     // Add carriage return
        uart_tx_string[message_len+3] <= 8'h0A;     // Add line feed
        uart_tx_process <= 1'b1;                    // Trigger UART transmission
    end
endtask

/*********************************************************************************************************/
function automatic logic [3:0] ascii_hex_to_nibble(input logic [7:0] c);
         if (c >= "0" && c <= "9") return c - "0";
    else if (c >= "a" && c <= "f") return c - "a" + 4'd10;
    else if (c >= "A" && c <= "F") return c - "A" + 4'd10;
    else
        return 4'hF; // invalid nibble
endfunction
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/



typedef enum logic [2:0] {
   STATE_INIT               = 3'b0,
   STATE_IDLE,
   STATE_PARSE_COMMAND,
   STATE_WAIT_FOR_SUBSTATE,
   STATE_PASS,
   STATE_FAIL,
   STATE_FIND_COMMA
} state_t;

always @(posedge clock) begin

    rx_fifo_read_en <= 1'b0;
    OE_Pin          <= 1'b1; // level shifter output enable

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
           comma_pos = -1;                
           for (i = 0; i < MAX_CMD_LENGTH; i = i + 1) begin
               if (command_buffer[i] == ",") begin
                   comma_pos = i;
                   break;
               end
           end
           command_state <= STATE_PARSE_COMMAND;
        end

        STATE_PARSE_COMMAND: begin
            for (i = 0; i < 5; i = i + 1) begin // niblett the 5 should be a constant!!!!!!!!!!!!!!!!!!
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
                command_state <= STATE_FAIL;
            end
        end

        STATE_WAIT_FOR_SUBSTATE: begin // note: only one substatemachine is active at any given time...
            if (substate_pb_i_write4_complete || substate_pb_read4_complete || substate_pb_adc4_complete) begin   // niblett renable ASAP!!!!!!!!!!!!!!!!!!!!!!
                substate_pb_i_write4_active <= 1'b0; // Deactivate the substate machine
                substate_pb_read4_active    <= 1'b0; // Deactivate the substate machine
                substate_pb_adc4_active     <= 1'b0; // Deactivate the substate machine
                command_state <= STATE_PASS; // Transition to STATE_PASS
            end
        end

        STATE_PASS: begin
           debug_hex_reg = 8'h56; // example
           send_debug_message(debug_hex_reg, {"P", "a", "s", "s", " ", "0", "x"}, 7);           
           command_state <= STATE_IDLE;
        end

        STATE_FAIL: begin
           debug_hex_reg = 8'h54; // example
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