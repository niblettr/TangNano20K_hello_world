module state_machines #(
    parameter CLOCK_FREQUENCY = 27000000 // System clock frequency in Hz
)(
    // Clock
    input       clock,                      // System clock

    // Activation Inputs
    input reg [1:0] lamp_card_reset_activate,
    input reg [1:0] substate_pb_i_write4_active,
    input reg [1:0] substate_pb_read4_active,
    input reg [1:0] substate_pb_adc4_active,

    // Completion Outputs
    output reg [1:0] lamp_card_reset_complete,
    output reg [1:0] substate_pb_i_write4_complete, 
    output reg [1:0] substate_pb_read4_complete,
    output reg [1:0] substate_pb_adc4_complete, 

    // Board Control Outputs
    output reg [3:0] BOARD_X,
    output reg [2:0] AddessPortPin,
    output reg       TestAddressP,
    output reg       RdP,
    output reg       WrP,
    output reg       LampResetPin,

    // Command Parameters
    input reg [7:0] command_param_data [0:4],

    // Data Ports
    output reg [7:0] Data_Out_Port,
    input      [7:0] Data_In_Port,
    output reg       data_dir,

    // Response Handling
    input reg [1:0] ResponsePending,
    input reg [7:0] ResponseBytes[0:7],
    input reg [3:0] ResponseByteCount

    // Debug (commented out for now)
    // output reg debug_hex_reg
);

`include "utils.v"

/*********************************************************************************************************/
parameter PORT_MUX = 8'h18; // 0x18 // --011--- /  Same Port address

/*********************************************************************************************************/

typedef enum logic [1:0] {
    DIR_INPUT  = 0,
    DIR_OUTPUT = 1          
} dir_mode_t;

typedef enum logic [1:0] {
    DISABLE  = 1,
    ENABLE   = 0          
} en_mode_t;


    // Define states for the new state machine
typedef enum logic [3:0] {
    SUBSTATE_PB_I_WRITE4_IDLE              = 4'b0000,
    SUBSTATE_PB_I_WRITE4_PRE_DELAY,
    SUBSTATE_PB_I_WRITE4_ASSERT_ADDRESS_ID,
    SUBSTATE_PB_I_WRITE4_WAIT_750N,
    SUBSTATE_PB_I_WRITE4_ASSERT_DATA,
    SUBSTATE_PB_I_WRITE4_ASSERT_WR_ENABLE,
    SUBSTATE_PB_I_WRITE4_RELEASE_WR,
    SUBSTATE_PB_I_WRITE4_RELEASE_DATA,
    SUBSTATE_PB_I_WRITE4_INC_CARD_ID_LOOP,
    SUBSTATE_PB_I_WRITE4_DONE
} substate_pb_i_write4_t;
substate_pb_i_write4_t substate_pb_i_write4      = SUBSTATE_PB_I_WRITE4_IDLE;
substate_pb_i_write4_t substate_pb_i_write4_next = SUBSTATE_PB_I_WRITE4_IDLE;

typedef enum logic [3:0] {
    SUBSTATE_PB_READ4_IDLE              = 4'b0000,
    SUBSTATE_PB_READ4_PRE_DELAY,
    SUBSTATE_PB_READ4_ASSERT_ADDRESS_ID,
    SUBSTATE_PB_READ4_WAIT_750N,
    SUBSTATE_PB_READ4_READ_DATA,
    SUBSTATE_PB_READ4_ASSERT_WR_ENABLE,
    SUBSTATE_PB_READ4_RELEASE_WR_ENABLE,
    SUBSTATE_PB_READ4_RELEASE_DATA,
    SUBSTATE_PB_READ4_INC_CARD_ID_LOOP,
    SUBSTATE_PB_READ4_REPLY,   
    SUBSTATE_PB_READ4_DONE              
} substate_pb_read4_t;

typedef enum logic [3:0] {
    SUBSTATE_PB_ADC4_IDLE              = 4'b0000,
    SUBSTATE_PB_ADC4_PRE_DELAY,
    SUBSTATE_PB_ADC4_ASSERT_ADDRESS_ID,
    SUBSTATE_PB_ADC4_WAIT_750N,
    SUBSTATE_PB_ADC4_ASSERT_DATA,
    SUBSTATE_PB_ADC4_ASSERT_WR_ENABLE,
    SUBSTATE_PB_ADC4_RELEASE_WR_ENABLE,
    SUBSTATE_PB_ADC4_RELEASE_DATA,
    SUBSTATE_PB_ADC4_INC_CARD_ID_LOOP,
    SUBSTATE_PB_ADC4_TEST_READ,
    SUBSTATE_PB_ADC4_DONE              
} substate_pb_adc4_t;


substate_pb_read4_t substate_pb_read4            = SUBSTATE_PB_READ4_IDLE;
substate_pb_read4_t substate_pb_read4_next       = SUBSTATE_PB_READ4_IDLE;

substate_pb_adc4_t substate_pb_adc4              = SUBSTATE_PB_ADC4_IDLE;
substate_pb_adc4_t substate_pb_adc4_next         = SUBSTATE_PB_ADC4_IDLE;

reg [4:0] substate_wait_counter  = 0; // 5 bits for up to 31 cycles
reg [2:0] wait_multiples         = 0;
reg [2:0] Board_ID_ptr           = 0;
reg [7:0] debug_hex_reg_test;

reg [7:0] Read_Data_buffer [4];
reg [7:0] ascii_out [7:0];

reg [7:0] uart_tx_response_string [0:20];
reg [7:0] uart_tx_response_string_len;
reg uart_tx_response_process        = 1'b0;
reg [7:0] reset_counter    = 8'b0; // 1-bit counter for reset delay

initial begin
          Read_Data_buffer[0] = 170;
          Read_Data_buffer[1] = 171;
          Read_Data_buffer[2] = 172;
          Read_Data_buffer[3] = 173;
end


always @(posedge clock) begin


    `include "InitCard.v"       // code for the Card initialisation 
    `include "Command_Write4.v"   


/************************************************************************************************************************/
/*
; (port) defines port address to be read (same for all boards)
; (buf_addr) defines address of data input buffer, data will be stored at contiguous
; addresses
; data buffer addresses must be in idata segment
[pb_read4](port,buf_adrr) x4 as there are 4 boards
MOV     P1,#BOARD_4 OR %port OR RD_ON
MOV     DPTR,#%buf_adrr
MOVX    A,@R0                 ; read data from bus
SETB    PB_RD                 ; read inactive
MOVX    @DPTR,A               ; store data to DPR
[loop 3 more times to do remaining cards (4 in total)]
*/


    //M_C pb_read4
    if (substate_pb_read4_active) begin
        case (substate_pb_read4)
            SUBSTATE_PB_READ4_IDLE: begin
                Board_ID_ptr  <= 0; // start from first card
                data_dir <= DIR_INPUT;
                substate_pb_read4 <= SUBSTATE_PB_READ4_ASSERT_ADDRESS_ID;
                end

            //MOV     P1,#BOARD_4 OR %port OR RD_ON // note its now 1,2,3 and 4 not 4,3,2 and 1
            SUBSTATE_PB_READ4_ASSERT_ADDRESS_ID: begin
                BOARD_X <= 4'b0001 << Board_ID_ptr; //BOARD_X = 1, 2, 4 or 8  
                AddessPortPin <= command_param_data[0][2:0];  // only use lowest 3 bits
                WrP <= DISABLE; // Write disabled
                RdP <= ENABLE; // Read enable
                wait_multiples <= 1;
                substate_pb_read4 <= SUBSTATE_PB_READ4_WAIT_750N;
                substate_pb_read4_next <= SUBSTATE_PB_READ4_READ_DATA;
            end

            SUBSTATE_PB_READ4_WAIT_750N: begin
                if(wait_multiples) begin
                    if (substate_wait_counter < 21) begin // Proceed after 21 cycles (~777ns) if clock = 20MHZ, 750ns can be achieved
                        substate_wait_counter <= substate_wait_counter + 1'b1;
                    end else begin                        
                        substate_wait_counter <= 0;
                        wait_multiples <= wait_multiples - 3'd1;
                    end
                end else begin
                   substate_pb_read4 <= substate_pb_read4_next;
                end
            end

            //MOVX    A,@R0                 ; read data from bus
            SUBSTATE_PB_READ4_READ_DATA: begin
               Read_Data_buffer[Board_ID_ptr] <= Data_In_Port;
               wait_multiples <= 1;
               substate_pb_read4_next <= SUBSTATE_PB_READ4_INC_CARD_ID_LOOP;
               substate_pb_read4 <= SUBSTATE_PB_READ4_WAIT_750N;
            end

            SUBSTATE_PB_READ4_INC_CARD_ID_LOOP: begin               
               if(Board_ID_ptr < (4 - 1)) begin   // 0->3 is 4 hence the -1
                  Board_ID_ptr <= Board_ID_ptr + 3'd1; 
                  substate_pb_read4 <= SUBSTATE_PB_READ4_ASSERT_ADDRESS_ID; // loop back round to do remaining cards
               end else begin
                   substate_pb_read4 <= SUBSTATE_PB_READ4_REPLY;
               end
            end

            SUBSTATE_PB_READ4_REPLY: begin
                //ResponseBytes[0:3] <= Read_Data_buffer[0:3];

                // hard coded debug responses 
                ResponseBytes[0] <= 8'hAA;
                ResponseBytes[1] <= 8'hBB;
                ResponseBytes[2] <= 8'hCC;
                ResponseBytes[3] <= 8'hDD;
                ResponseByteCount <= 4;
                ResponsePending   <= 1;
                substate_pb_read4 <= SUBSTATE_PB_READ4_DONE;

            end

            SUBSTATE_PB_READ4_DONE: begin
                ResponsePending <= 0;
                substate_pb_read4_complete <= 1'b1;   // Indicate substate_pb_read4 completion
                substate_pb_read4 <= SUBSTATE_PB_READ4_IDLE;
            end

        endcase
    end else begin
        substate_pb_read4_complete <= 1'b0; // Clear the flag when substate_pb_i_read4 is inactive
    end // end of if (substate_pb_i_read4_active) begin


/************************************************************************************************************************/



/************************************************************************************************************************/
/*
; mux_inp                   defines Analog Muxer channel
; buf_addr                  defines address for storing adc output from 1. board
; buf_addr + offset1        defines address for storing adc output from 2. board
; buf_addr + offset2        defines address for storing adc output from 3. board
; buf_addr + offset3        defines address for storing adc output from 4. board
[pb_adc4_16] (mux_inp, buf_addr, offset1, offset2, offset3)
(
MOV     P1,#BOARD_ALL OR PORT_MUX OR CTR_OFF
MOV     A,#%mux_inp
;Send the required data to the multiplexer latches...
MOVX    @R0,A                 ; output mux-port
CLR     DIR_OUT               ; switch data on Phase Bus
CLR     PB_WR                 ; SETB WR activ
NOP                           ; make pulse longer
SETB    PB_WR                 ; SETB WR passiv
NOP                           ; wait until mux-out stable
NOP
NOP
NOP
;Start the ADC conversion now mux-out is stable...
CLR     PB_WR                 ; SETB WR activ
NOP                           ; make pulse longer
SETB    PB_WR                 ; SETB WR passiv
;Now read the data back from all the boards...
SETB    DIR_OUT               ; Switch Phase Bus for input
LOOP4
MOV     P1,#BOARD_1 OR PORT_ADC_HIGH OR RD_ON
MOV     DPTR,#%buf_addr
MOVX    A,@R0                 ; read data from bus
SETB    PB_RD                 ; read inactive
MOVX    @DPTR,A               ; store data to DPR
MOV     P1,#BOARD_1 OR PORT_ADC_LOW OR RD_ON
INC     DPTR
MOVX    A,@R0                 ; read data from bus
SETB    PB_RD                 ; read inactive
MOVX    @DPTR,A               ; store data to DPR
GOTO LOOP4
*/

/*
    // handles the pb_adc4_16 & pb_adc4_8
    // doing pb_adc4_16 first
    if (substate_pb_adc4_active) begin
        case (substate_pb_adc4)
            SUBSTATE_PB_ADC4_IDLE: begin
                Board_ID_ptr <= 0;
                data_dir        <= 1; // set data_port to output mode
                wait_multiples <= 3;
                substate_pb_adc4_next = SUBSTATE_PB_ADC4_DONE;
                substate_pb_adc4 <= SUBSTATE_PB_ADC4_WAIT_750N;
                end

            //MOV     P1,#BOARD_ALL OR PORT_MUX OR CTR_OFF
            //#define BOARD_ALL            0x05 // -----101
            SUBSTATE_PB_ADC4_ASSERT_ADDRESS_ID: begin
                BOARD_X <= 5;//BOARD_ALL;
                AddessPortPin <= PORT_MUX;
                WrP <= DISABLE; // CTR_OFF in the assembler
                RdP <= DISABLE; // CTR_OFF in the assembler
                wait_multiples <= 1;
                substate_pb_adc4 <= SUBSTATE_PB_ADC4_WAIT_750N;
                substate_pb_adc4_next <= SUBSTATE_PB_ADC4_DONE; // <<<<<<<<<<<<<<<<<<<<<<
            end

            SUBSTATE_PB_ADC4_WAIT_750N: begin
                if(wait_multiples) begin
                    if (substate_wait_counter < 21) begin // Proceed after 21 cycles (~777ns) if clock = 20MHZ, 750ns can be achieved
                        substate_wait_counter <= substate_wait_counter + 1'b1;
                    end else begin                        
                        substate_wait_counter <= 0;
                        wait_multiples <= wait_multiples - 3'd1;
                    end
                end else begin
                   substate_pb_adc4 <= substate_pb_adc4_next;
                end
            end
            SUBSTATE_PB_ADC4_DONE: begin
                if(CommandType == 0) begin

                // send response back
                hex_to_ascii(Read_Data_buffer, 4, ascii_out);
                uart_tx_string[0:7]  <={"a", "d", "c", "4", "_", "1", "6", ","};
                uart_tx_string[8:17] <={ascii_out[0], ascii_out[1], ascii_out[2], ascii_out[3], 
                                        ascii_out[4], ascii_out[5], ascii_out[6], ascii_out[7], 8'h0D, 8'h0A};

                uart_tx_string_len  <= 18;
                uart_tx_process     <= 1'b1;   // Trigger UART transmission

                   //send_debug_message(debug_hex_reg, {"A", "D", "C", "4", "_", "1", "6", " ", "0", "x"}, 10);
                end else begin
                  // send_debug_message(debug_hex_reg, {"A", "D", "C", "4", "_", "8", " ", "0", "x"}, 9);

                hex_to_ascii(Read_Data_buffer, 4, ascii_out);
                uart_tx_string[0:6]  <={"a", "d", "c", "4", "_", "8", ","};
                uart_tx_string[7:16] <={ascii_out[0], ascii_out[1], ascii_out[2], ascii_out[3], 
                                        ascii_out[4], ascii_out[5], ascii_out[6], ascii_out[7], 8'h0D, 8'h0A};

                uart_tx_string_len  <= 17;
                uart_tx_process     <= 1'b1;   // Trigger UART transmission


                end

                substate_pb_read4_complete <= 1'b1;   // Indicate substate_pb_adc4 completion
                substate_pb_adc4 <= SUBSTATE_PB_ADC4_IDLE;
            end

        endcase
    end else begin
        substate_pb_adc4_complete <= 1'b0; // Clear the flag when substate_pb_i_read4 is inactive
    end // end of if (substate_pb_i_read4_active) begin

*/
/************************************************************************************************************************/
end



endmodule