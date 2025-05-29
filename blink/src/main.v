module Top_module( 
    input  clock,          // System Clock 27MHz            Pin4
    //output reg [7:0] DataPortPins,    // Data Port          Pin73,Pin74,Pin75,Pin85,Pin77,Pin15,Pin16,Pin27
    inout [7:0] DataPortPins,
    output reg [2:0] AddessPortPin,   // Address Port       Pin28,Pin25,Pin26
    output reg [3:0] B_ID_pins,// B3,B2,B1,B0,              Pin29, Pin71, Pin72, Pin29
    output reg TestAddressP,   //                           Pin30
    output reg RdP,            // Read Enable               Pin31
    output reg WrP,            // write Enable              Pin17
    output reg LampResetPin,   //                           Pin20
    output reg OE_Pin,     // LevelShifter Enable           Pin19
    output Uart_TX_Pin,    // Transmit pin of UART          Pin55
    input  Uart_RX_Pin,    // Receive pin of UART           Pin49
    output Debug_Pin,      // Debug toggle                  Pin76
    output Debug_Pin2      // Debug toggle                  Pin80
);

    //reg TopLevelDebug2  = 0; // commented out for time being to remove warning

    reg [7:0] data_out_pins;
    wire [7:0] data_in_pins;
    reg data_dir; // 1 = output, 0 = input


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
    reg [15:0] debug_16hex_reg = "AA";

    reg [7:0] debug_hex_ascii [0:1] = "AA";
/**********************************************************************/
task hex_to_ascii;
    input [7:0] hex_value;   // Input hex value
    begin
        logic [7:0] high, low;
        // Append the hex value as ASCII characters
        high = (hex_value[7:4] < 10) ? (hex_value[7:4] + 8'd48) : (hex_value[7:4] - 4'd10 + 8'd65);
        low  = (hex_value[3:0] < 10) ? (hex_value[3:0] + 8'd48) : (hex_value[3:0] - 4'd10 + 8'd65);

        debug_hex_ascii[0] = high;
        debug_hex_ascii[1] = low;
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

function automatic logic [3:0] ascii_hex_to_nibble(input logic [7:0] c);
         if (c >= "0" && c <= "9") return c - "0";
    else if (c >= "a" && c <= "f") return c - "a" + 4'd10;
    else if (c >= "A" && c <= "F") return c - "A" + 4'd10;
    else
        return 4'hF; // invalid nibble (optional: flag error)
endfunction
/**********************************************************************/

    /********** UART Transmission **********/
    reg tx_fifo_write_en;
    reg [7:0] tx_fifo_data_in = 8'b0;
    /********** UART Reception **********/
    reg rx_fifo_read_en;
    reg [7:0] rx_fifo_data_out = 8'b0;
    wire rx_fifo_empty;
    wire rx_sentence_received;
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

        .rx_sentence_received(rx_sentence_received)       // Connect rx_sentence_received
        //.Debug_uart(Debug_uart)
    );

parameter MAX_CMD_LENGTH = 30;
parameter CMD_LENGTH = 11; // "pb_i_write, or pb_i_read,"
reg [7:0] command_buffer [0:32-1]; // Buffer to store the command

wire [8*CMD_LENGTH:0] command_word = {command_buffer[0], command_buffer[1], command_buffer[2], command_buffer[3],
                                      command_buffer[4], command_buffer[5], command_buffer[6], command_buffer[7],
                                      command_buffer[8], command_buffer[9], command_buffer[10]};

reg [5:0] command_index = 0;               // Index for the command buffer
reg [5:0] command_len = 0;
    // Define FSM states with meaningful names
    typedef enum logic [2:0] {
        STATE_INIT             = 3'b000,
        STATE_IDLE             = 3'b001,
        STATE_PARSE_COMMAND    = 3'b010,
        STATE_WAIT             = 3'b011,
        STATE_PASS             = 3'b100,
        STATE_FAIL             = 3'b101,
        STATE_COMMA            = 3'b110,
        STATE_PARSE_PARAMS     = 3'b111
    } state_t;

reg [2:0] command_state = STATE_INIT;      // State machine for command handling

reg uart_rx_previous_empty = 1'b0; // Flag to track if RX processing is in progress
reg uart_tx_process        = 1'b0;

reg [2:0] uart_tx_state = 3'b000; // State machine for UART string transmission
reg [7:0] reset_counter = 8'b0; // 1-bit counter for reset delay

// Define states for the new state machine
typedef enum logic [3:0] {
    SUBSTATE_PB_I_WRITE4_IDLE              = 4'b0000,
    SUBSTATE_PB_I_WRITE4_PRE_DELAY         = 4'b0001,
    SUBSTATE_PB_I_WRITE4_ASSERT_ADDRESS_ID = 4'b0010,
    SUBSTATE_PB_I_WRITE4_WAIT_750N         = 4'b0011,
    SUBSTATE_PB_I_WRITE4_ASSERT_DATA       = 4'b0100,
    SUBSTATE_PB_I_WRITE4_ASSERT_WR_ENABLE  = 4'b0101,
    SUBSTATE_PB_I_WRITE4_RELEASE_WR_ENABLE = 4'b0110,
    SUBSTATE_PB_I_WRITE4_RELEASE_DATA      = 4'b0111,
    SUBSTATE_PB_I_WRITE4_INC_CARD_ID_LOOP  = 4'b1000,
    SUBSTATE_PB_I_WRITE4_TEST_READ         = 4'b1001,
    SUBSTATE_PB_I_WRITE4_DONE              = 4'b1010
} substate_pb_i_write4_t;

substate_pb_i_write4_t substate_pb_i_write4      = SUBSTATE_PB_I_WRITE4_IDLE; // State variable for the new state machine
substate_pb_i_write4_t substate_pb_i_write4_next = SUBSTATE_PB_I_WRITE4_IDLE; // State variable for the new state machine

//reg [3:0] substate_pb_read4      = SUBSTATE_IDLE; // State variable for the new state machine
//reg [3:0] substate_pb_read4_next = SUBSTATE_IDLE; // State variable for the new state machine

//////////////////////////////////////////////////////////////////////////////////////////////////////////
reg substate_pb_i_write4_active     = 1'b0;      // Flag to indicate if the substate machine is active
reg substate_pb_i_write4_complete   = 1'b0;  // Flag to indicate substate completion

reg substate_pb_read4_active        = 1'b0;         // Flag to indicate if the substate machine is active
reg substate_pb_read4_complete      = 1'b0;     // Flag to indicate substate completion

reg substate_pb_adc4_16_active      = 1'b0;       // Flag to indicate if the substate machine is active
reg substate_pb_adc4_16_complete    = 1'b0;   // Flag to indicate substate completion

reg substate_pb_adc4_8_active       = 1'b0;        // Flag to indicate if the substate machine is active
reg substate_pb_adc4_8_complete     = 1'b0;    // Flag to indicate substate completion
//////////////////////////////////////////////////////////////////////////////////////////////////////////

reg [7:0] command_param_data [0:4]; // 5 bytes (10 hex chars)
integer i;                          // general purpose 
integer comma_pos;
reg [4:0] substate_wait_counter = 0; // 5 bits for up to 31 cycles
reg [2:0] wait_multiples         = 0;
reg [2:0] Card_ID                = 0;
reg [1:0] cmd_type               = 0;


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
        STATE_INIT: begin // we want to reset fifo here then move on..
           if(reset_counter < 100) begin
              reset_counter  <= reset_counter + 1'b1;
              LampResetPin   <= 1'b1;        // hold in reset
              data_dir       <= 0;  // input
              data_out_pins  <= 8'b0;
              AddessPortPin  <= 3'b0;
              TestAddressP   <= 1'b01;
              RdP            <= 1'b1;
              WrP            <= 1'b1;
              OE_Pin         <= 1'b0;   // Enable pin of LevelShifter
           end else begin
              LampResetPin   <= 1'b0;   // release from reset state
              OE_Pin         <= 1'b1;   // Enable pin of LevelShifter
              command_state <= STATE_IDLE;
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
                command_state <= STATE_COMMA; // Move to command processing state
            end
        end

        STATE_COMMA: begin
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
            if (command_word == "pb_i_write,") begin
                command_state <= STATE_PARSE_PARAMS; // Transition to a wait state
                cmd_type = 0; // write
             end else if (command_word == "pb_i__read,") begin
                command_state <= STATE_PARSE_PARAMS; // Transition to a wait state
                cmd_type = 1; // read
            end else begin
                debug_16hex_reg = 8'hAA; // example
                command_state <= STATE_FAIL;
            end
        end

        STATE_PARSE_PARAMS: begin
            for (i = 0; i < 5; i = i + 1) begin
               logic [3:0] high_nibble, low_nibble;

               high_nibble = ascii_hex_to_nibble(command_buffer[comma_pos + 1 + i*2]);
               low_nibble  = ascii_hex_to_nibble(command_buffer[comma_pos + 2 + i*2]);

               command_param_data[i] = {high_nibble, low_nibble};
            end
            substate_pb_i_write4_active <= 1'b1; // Activate the new state machine
            command_state <= STATE_WAIT; // Transition to a wait state
        end

        STATE_WAIT: begin // note: only one substatemachine is active at any given time...
            if (substate_pb_i_write4_complete) begin
                substate_pb_i_write4_active <= 1'b0; // Deactivate the substate machine
                command_state <= STATE_PASS; // Transition to STATE_PASS
            end
        end

        STATE_PASS: begin
           debug_hex_reg = 8'h56; // example
           send_debug_message(debug_hex_reg, {"P", "a", "s", "s", " ", "0", "x"}, 7);
           command_state <= STATE_IDLE;
        end

        STATE_FAIL: begin
           
           //send_debug_message(debug_hex_reg, {"F", "a", "i", "l", "e", "d", " ", "0", "x", debug_16hex_reg}, 11);
           command_state <= STATE_IDLE;
        end

        default: begin
            command_state <= STATE_IDLE; // Reset to idle state
        end
    endcase
/************************************************************************************************************************/



/************************************************************************************************************************/
    if (substate_pb_i_write4_active) begin
        case (substate_pb_i_write4)
            SUBSTATE_PB_I_WRITE4_IDLE: begin
                Card_ID <= 0;
                data_dir        <= 1; // set data_port to output mode
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_PRE_DELAY;
                end

            SUBSTATE_PB_I_WRITE4_PRE_DELAY: begin // initial delay to account for first liner MOV     R1,#%buf_addr                
                wait_multiples <= 1;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_WAIT_750N;
                substate_pb_i_write4_next <= SUBSTATE_PB_I_WRITE4_ASSERT_ADDRESS_ID;  
            end

            SUBSTATE_PB_I_WRITE4_ASSERT_ADDRESS_ID: begin // equivalent to P1,#BOARD_4 OR %port OR CTR_OFF, might need a 750ns delay prior to this.........
                B_ID_pins <= 4'b0001 << Card_ID; //B_ID_pins = 1, 2, 4 or 8  
                AddessPortPin <= command_param_data[0][2:0];  // only use lowest 3 bits
                WrP <= 1; // CTR_OFF in the assembler
                RdP <= 1; // CTR_OFF in the assembler
                wait_multiples <= 4;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_WAIT_750N;
                substate_pb_i_write4_next <= SUBSTATE_PB_I_WRITE4_ASSERT_DATA;
            end

            SUBSTATE_PB_I_WRITE4_WAIT_750N: begin
                if(wait_multiples) begin
                    if (substate_wait_counter < 21) begin // Proceed after 21 cycles (~777ns) if clock = 20MHZ, 750ns can be achieved
                        substate_wait_counter <= substate_wait_counter + 1'b1;
                    end else begin                        
                        substate_wait_counter <= 0;
                        wait_multiples <= wait_multiples - 3'd1;
                    end
                end else begin
                   substate_pb_i_write4 <= substate_pb_i_write4_next;
                end
            end

            SUBSTATE_PB_I_WRITE4_ASSERT_DATA: begin
                data_out_pins <= command_param_data[Card_ID +1]; // BYTE 0 = ADDRESS HENCE THE +1
                wait_multiples <= 1;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_WAIT_750N;
                substate_pb_i_write4_next <= SUBSTATE_PB_I_WRITE4_ASSERT_WR_ENABLE;
            end

            SUBSTATE_PB_I_WRITE4_ASSERT_WR_ENABLE: begin
                WrP <= 0; // active low
                wait_multiples <= 2;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_WAIT_750N;
                substate_pb_i_write4_next <= SUBSTATE_PB_I_WRITE4_RELEASE_WR_ENABLE;
            end

            SUBSTATE_PB_I_WRITE4_RELEASE_WR_ENABLE: begin
                WrP <= 1;
                wait_multiples <= 2;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_WAIT_750N;
                substate_pb_i_write4_next <= SUBSTATE_PB_I_WRITE4_RELEASE_DATA;
            end

            SUBSTATE_PB_I_WRITE4_RELEASE_DATA: begin
                data_dir        <= 0;  // input/RELEASE THE DATA PINS
                wait_multiples <= 1;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_WAIT_750N;
                substate_pb_i_write4_next <= SUBSTATE_PB_I_WRITE4_INC_CARD_ID_LOOP;
            end

            SUBSTATE_PB_I_WRITE4_INC_CARD_ID_LOOP: begin               
               if(Card_ID < (4 - 1)) begin   // 0->3 is 4 hence the -1  
                  debug_hex_reg =  Card_ID;       
                  Card_ID <= Card_ID + 3'd1; 
                  substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_ASSERT_ADDRESS_ID; // loop back round to do remaining cards
               end else begin
                   substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_DONE;
               end
            end

            SUBSTATE_PB_I_WRITE4_TEST_READ: begin
              // data_bytes[0] <= data_out_pins;
               debug_hex_reg = data_in_pins;
               send_debug_message(debug_hex_reg, {"R", "e", "a", "d", " ", "0", "x"}, 7);
               substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_DONE;
            end


            SUBSTATE_PB_I_WRITE4_DONE: begin
                substate_pb_i_write4_complete <= 1'b1;   // Indicate substate_pb_i_write4 completion
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_IDLE;
            end
        endcase
    end else begin
        substate_pb_i_write4_complete <= 1'b0; // Clear the flag when substate_pb_i_write4 is inactive
    end
end

/********** Continuous Assignment **********/
assign Debug_Pin = rx_sentence_received;
assign Debug_Pin2 = clock;
//assign Debug_Pin = Debug_uart;
//assign Debug_Pin = Debug_spi;
//assign Debug_Pin = TopLevelDebug;

// Assign bidirectional port with tristate buffer behavior
assign DataPortPins = (data_dir) ? data_out_pins : 8'bz;  // drive data_out_pins if output
assign data_in_pins = DataPortPins;                      // read pins as input

endmodule