module Top_module( 
    input  clock,          // System Clock 27MHz            Pin4
    output reg [7:0] DataPortPins,    // Data Port          Pin73,Pin74,Pin75,Pin85,Pin77,Pin15,Pin16,Pin27
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

    reg TopLevelDebug2  = 0;


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

        .rx_sentence_received(rx_sentence_received),       // Connect rx_sentence_received
        .Debug_uart(Debug_uart)
    );

parameter CMD_LENGTH = 11; // "pb_i_write,"
reg [7:0] command_buffer [0:32-1]; // Buffer to store the command

wire [8*CMD_LENGTH:0] command_word = {command_buffer[0], command_buffer[1], command_buffer[2], command_buffer[3], command_buffer[4],
                                      command_buffer[5], command_buffer[6], command_buffer[7], command_buffer[8], command_buffer[9], command_buffer[10]};

reg [5:0] command_index = 0;               // Index for the command buffer
reg [5:0] command_len = 0;
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

// Define states for the new state machine
typedef enum logic [3:0] {
    SUBSTATE_IDLE      = 4'b0000,
    SUBSTATE_TASK1     = 4'b0001,
    SUBSTATE_TASK2     = 4'b0010,
    SUBSTATE_WAIT_750N = 4'b0011,
    SUBSTATE_TASK4     = 4'b0100,
    SUBSTATE_TASK5     = 4'b0101,
    SUBSTATE_TASK6     = 4'b0110,
    SUBSTATE_TASK7     = 4'b0111,
    SUBSTATE_TASK8     = 4'b1000,
    SUBSTATE_DONE      = 4'b1001
} substate_t;

reg [3:0] substate      = SUBSTATE_IDLE; // State variable for the new state machine
reg [3:0] substate_next = SUBSTATE_IDLE; // State variable for the new state machine

reg substate_active = 1'b0;         // Flag to indicate if the substate machine is active
reg substate_done = 1'b0; // Flag to indicate substate completion


reg [7:0] data_bytes [0:4]; // 5 bytes (10 hex chars)
integer i;
integer comma_pos;
//reg [4:0] substate_wait_counter = 0; // 5 bits for up to 31 cycles
reg [31:0] substate_wait_counter = 0; // 5 bits for up to 31 cycles
reg [2:0] wait_multiples         = 0;
reg [2:0] Card_ID                = 0;


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
              reset_counter  <= reset_counter + 1'b1;
              LampResetPin       <= 1'b1;        // hold in reset
              DataPortPins   <= 8'b0;
              AddessPortPin  <= 3'b0;
              TestAddressP   <= 1'b01;
              RdP            <= 1'b1;
              WrP            <= 1'b1;
              OE_Pin         <= 1'b0;   // Enable pin of LevelShifter
           end else begin
              LampResetPin       <= 1'b0;   // release from reset state
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
                //TopLevelDebug2 <= ~TopLevelDebug2;
              
            end else if (rx_fifo_empty) begin
              uart_rx_previous_empty <= 1'b1;
            end

            if(rx_sentence_received) begin
                command_len <= command_index;
                command_index <= 3'b0;        // reset to zero
                command_state <= STATE_PARSE; // Move to command processing state
            end
        end

        STATE_PARSE: begin
            if (command_word == "pb_i_write,") begin
                substate_active <= 1'b1; // Activate the new state machine
                command_state <= STATE_WAIT; // Transition to a wait state
            end else begin
                command_state <= STATE_FAIL;
            end
        end

        STATE_WAIT: begin
            if (substate_done) begin
                substate_active <= 1'b0; // Deactivate the substate machine
                command_state <= STATE_PASS; // Transition to STATE_PASS
            end
        end

        STATE_PASS: begin
           //debug_hex_reg = 8'h55; // example
           //send_debug_message(debug_hex_reg, {"P", "a", "s", "s", " ", "0", "x"}, 7);
           command_state <= STATE_IDLE;
        end

        STATE_FAIL: begin
           debug_hex_reg = 8'hAA; // example
           send_debug_message(debug_hex_reg, {"F", "a", "i", "l", "e", "d", " ", "0", "x", debug_16hex_reg}, 11);
           command_state <= STATE_IDLE;
        end

        default: begin
            command_state <= STATE_IDLE; // Reset to idle state
        end
    endcase
/************************************************************************************************************************/
/************************************************************************************************************************/
/*
#define PORT_RED             0x00 // --000---
#define PORT_AMB             0x08 // --001---
#define PORT_GRE             0x10 // --010---
#define PORT_ADC_LOW         0x18 // --011--- \_ Same Port address
#define PORT_MUX             0x18 // --011--- /  Same Port address
#define PORT_ADC_HIGH        0x20 // --100---
#define PORT_TEST            0x28 // --101---
#define PORT_RESERVED6       0x30 // --110--- Write to turn mimics ON?
#define PORT_RESERVED7       0x38 // --101--- Write to turn mimics OFF?

asm_pb_i_write4_output_request:
%pb_i_write4 (PORT_RED, req_red_new) // writes 4 consecutive bytes on data ports
%pb_i_write4 (PORT_AMB, req_amb_new) // writes 4 consecutive bytes on data ports
%pb_i_write4 (PORT_GRE, req_gre_new) // writes 4 consecutive bytes on data ports
*/
    if (substate_active) begin
        case (substate)
            SUBSTATE_IDLE: begin
                comma_pos = -1;                
                for (i = 0; i < 20; i = i + 1) begin     // the 20 is horrible.....FIX ASAP!!!
                    if (command_buffer[i] == ",") begin
                        comma_pos = i;
                        break;
                    end
                end
                substate <= SUBSTATE_TASK1;

            end

            SUBSTATE_TASK1: begin
                TopLevelDebug2 <= ~TopLevelDebug2; // Example: Toggle a debug signal
                //command_word = pb_i_write,ffaabbccddee  ff = 1 byte colour,aabbccddee = 4 data bytes
                for (i = 0; i < 5; i = i + 1) begin
                    data_bytes[i] = ((command_buffer[comma_pos+1 + i*2] > "9" ? command_buffer[comma_pos+1 + i*2] - "a" + 4'd10 : command_buffer[comma_pos+1 + i*2] - "0") << 4)
                                  |  (command_buffer[comma_pos+2 + i*2] > "9" ? command_buffer[comma_pos+2 + i*2] - "a" + 4'd10 : command_buffer[comma_pos+2 + i*2] - "0");           
                end
                substate <= SUBSTATE_TASK2;
                Card_ID <= 0;
            end

            SUBSTATE_TASK2: begin // assert address & board ID bits

                if(Card_ID == 0) begin
                    send_debug_message(debug_hex_reg, {"0", "0", "0", "0", " ", "0", "x"}, 7);
                    B_ID_pins <= 1;
                end else if (Card_ID == 1) begin
                    send_debug_message(debug_hex_reg, {"1", "0", "0", "0", " ", "0", "x"}, 7);
                    B_ID_pins <= 2;
                end else if (Card_ID == 2) begin
                    send_debug_message(debug_hex_reg, {"2", "0", "0", "0", " ", "0", "x"}, 7);
                    B_ID_pins <= 4;
                end else begin
                    send_debug_message(debug_hex_reg, {"4", "0", "0", "0", " ", "0", "x"}, 7);
                    B_ID_pins <= 8;
                end


                AddessPortPin <= data_bytes[0]; // no need to mask out bit 4 ?
                wait_multiples <= 4;
                substate <= SUBSTATE_WAIT_750N;
                substate_next <= SUBSTATE_TASK4;
            end

            SUBSTATE_WAIT_750N: begin
                if(wait_multiples) begin
                    //if (substate_wait_counter < 13500000) begin  //substate <= SUBSTATE_TASK2; 
                    if (substate_wait_counter < 21) begin // Proceed after 21 cycles (~777ns)
                        substate_wait_counter <= substate_wait_counter + 1'b1;
                    end else begin                        
                        substate_wait_counter <= 0;
                        wait_multiples <= wait_multiples -1;
                    end
                end else begin
                   substate <= substate_next;
                end
            end

            SUBSTATE_TASK4: begin // assert data phase
                DataPortPins <= data_bytes[Card_ID +1]; // BYTE 0 = ADDRESS HENCE THE +1
                wait_multiples <= 1;
                substate <= SUBSTATE_WAIT_750N;
                substate_next <= SUBSTATE_TASK5;

            end

            SUBSTATE_TASK5: begin // ASSERT WRITE PIN PHASE
                WrP <= 0; // ACTIVE LOW
                wait_multiples <= 2;
                substate <= SUBSTATE_WAIT_750N;
                substate_next <= SUBSTATE_TASK6;
                send_debug_message(debug_hex_reg, {"P", "a", "s", "s", " ", "0", "x"}, 7);
            end

            SUBSTATE_TASK6: begin // RELEASE WRITE PIN PHASE
                WrP <= 1;
                wait_multiples <= 2;
                substate <= SUBSTATE_WAIT_750N;
                substate_next <= SUBSTATE_TASK7;
            end

            SUBSTATE_TASK7: begin // 
                DataPortPins <= 255; // RELEASE THE DATA PINS
                wait_multiples <= 1;
                substate <= SUBSTATE_WAIT_750N;
                substate_next <= SUBSTATE_TASK8;
            end

            SUBSTATE_TASK8: begin
               
               if(Card_ID < 4 - 1) begin   // 0->3 is 4 hence the -1  
                  debug_hex_reg =  Card_ID;       
                  Card_ID <= Card_ID +1;
                  //send_debug_message(debug_hex_reg, {"P", "a", "s", "s", " ", "0", "x"}, 7);
                  substate <= SUBSTATE_TASK2; // loop back round to do remaining cards
               end else begin
                   substate <= SUBSTATE_DONE;
                   //send_debug_message(debug_hex_reg, {"P", "a", "s", "s", " ", "0", "x"}, 7);
               end
            end


            SUBSTATE_DONE: begin
                //send_debug_message(debug_hex_reg, {"P", "a", "s", "s", " ", "0", "x"}, 7);
                // Indicate substate completion
                substate_done <= 1'b1;   // Indicate substate completion
                substate <= SUBSTATE_IDLE;
            end
        endcase
    end else begin
        substate_done <= 1'b0; // Clear the flag when substate is inactive
    end
end





//always @(posedge clock) begin

//end


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
assign Debug_Pin = rx_sentence_received;
assign Debug_Pin2 = TopLevelDebug2;
//assign Debug_Pin = Debug_uart;
//assign Debug_Pin = Debug_spi;
//assign Debug_Pin = TopLevelDebug;

endmodule