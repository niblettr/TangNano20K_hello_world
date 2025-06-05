module state_machines #(
    parameter CLOCK_FREQUENCY = 27000000 // System clock frequency in Hz
)(
    input       clock,                      // System clock
    input       substate_pb_i_write4_active,
    input       substate_pb_read4_active,
    input       substate_pb_adc4_active 
);
/*********************************************************************************************************/
parameter PORT_MUX = 8'h18; // 0x18 // --011--- /  Same Port address

/*********************************************************************************************************/

typedef enum logic [1:0] {
    DIR_INPUT  = 0,
    DIR_OUTPUT = 1          
} dir_mode_t;

typedef enum logic [1:0] {
    DISABLE  = 0,
    ENABLE   = 1          
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
    SUBSTATE_PB_READ4_TEST_READ,
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

substate_pb_i_write4_t substate_pb_i_write4      = SUBSTATE_PB_I_WRITE4_IDLE;
substate_pb_i_write4_t substate_pb_i_write4_next = SUBSTATE_PB_I_WRITE4_IDLE;

substate_pb_read4_t substate_pb_read4            = SUBSTATE_PB_READ4_IDLE;
substate_pb_read4_t substate_pb_read4_next       = SUBSTATE_PB_READ4_IDLE;

substate_pb_adc4_t substate_pb_adc4              = SUBSTATE_PB_ADC4_IDLE;
substate_pb_adc4_t substate_pb_adc4_next         = SUBSTATE_PB_ADC4_IDLE;

reg [4:0] substate_wait_counter  = 0; // 5 bits for up to 31 cycles
reg [2:0] wait_multiples         = 0;
reg [2:0] Board_ID_ptr           = 0;
reg data_dir; // 1 = output, 0 = input





always @(posedge clock) begin
/*
[pb_i_write4](port,buf_addr) x4 as there are 4 boards
MOV     R1,#%buf_addr

MOV     P1,#BOARD_4 OR %port OR CTR_OFF
MOV     A,@R1                 ; load output data to ACC
MOVX    @R0,A                 ; load data to output latch
CLR     DIR_OUT               ; output driver on
CLR     PB_WR                 ; activate WR-line
INC     R1                    ; adjust pointer to next item
SETB    PB_WR                 ; WR-line inactive
SETB    DIR_OUT               ; output driver off
[loop 3 more times to do remaining cards (4 in total)]
*/
/*
    if (substate_pb_i_write4_active) begin
        case (substate_pb_i_write4)
            SUBSTATE_PB_I_WRITE4_IDLE: begin
                Board_ID_ptr  <= 0;
                data_dir <= DIR_OUTPUT;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_PRE_DELAY;
                end

            SUBSTATE_PB_I_WRITE4_PRE_DELAY: begin // initial delay to account for first liner MOV     R1,#%buf_addr                
                wait_multiples <= 1;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_WAIT_750N;
                substate_pb_i_write4_next <= SUBSTATE_PB_I_WRITE4_ASSERT_ADDRESS_ID;
            end

            //MOV     P1,#BOARD_4 OR %port OR CTR_OFF // note its now 1,2,3 and 4 not 4,3,2 and 1
            SUBSTATE_PB_I_WRITE4_ASSERT_ADDRESS_ID: begin
                BOARD_X <= 4'b0001 << Board_ID_ptr; //BOARD_X = 1, 2, 4 or 8  
                AddessPortPin <= command_param_data[0][2:0];  // only use lowest 3 bits
                WrP <= DISABLE; // CTR_OFF in the assembler
                RdP <= DISABLE; // CTR_OFF in the assembler
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

            //MOVX    @R0,A                 ; load data to output latch
            //CLR     DIR_OUT               ; output driver on
            SUBSTATE_PB_I_WRITE4_ASSERT_DATA: begin
                Data_Out_Port <= command_param_data[Board_ID_ptr +1]; // BYTE 0 = ADDRESS HENCE THE +1
                wait_multiples <= 1;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_WAIT_750N;
                substate_pb_i_write4_next <= SUBSTATE_PB_I_WRITE4_ASSERT_WR_ENABLE;
            end

            //CLR     PB_WR                 ; activate WR-line
            SUBSTATE_PB_I_WRITE4_ASSERT_WR_ENABLE: begin
                WrP <= ENABLE; // active low
                wait_multiples <= 2;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_WAIT_750N;
                substate_pb_i_write4_next <= SUBSTATE_PB_I_WRITE4_RELEASE_WR;
            end

            SUBSTATE_PB_I_WRITE4_RELEASE_WR: begin
                WrP <= DISABLE;
                wait_multiples <= 2;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_WAIT_750N;
                substate_pb_i_write4_next <= SUBSTATE_PB_I_WRITE4_RELEASE_DATA;
            end

            //SETB    DIR_OUT               ; output driver off
            SUBSTATE_PB_I_WRITE4_RELEASE_DATA: begin
                data_dir        <= DIR_INPUT;  // back to input mode
                wait_multiples <= 1;
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_WAIT_750N;
                substate_pb_i_write4_next <= SUBSTATE_PB_I_WRITE4_INC_CARD_ID_LOOP;
            end

            SUBSTATE_PB_I_WRITE4_INC_CARD_ID_LOOP: begin               
               if(Board_ID_ptr < (4 - 1)) begin   // 0->3 is 4 hence the -1  
                  debug_hex_reg =  Board_ID_ptr;       
                  Board_ID_ptr <= Board_ID_ptr + 3'd1; 
                  substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_ASSERT_ADDRESS_ID; // loop back round to do remaining cards
               end else begin
                   substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_DONE;
               end
            end

            SUBSTATE_PB_I_WRITE4_DONE: begin
                send_debug_message(debug_hex_reg, {"W", "r", "i", "t", "e", " ", "0", "x"}, 8);
                substate_pb_i_write4_complete <= 1'b1;   // Indicate substate_pb_i_write4 completion
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_IDLE;
            end
        endcase
    end else begin
        substate_pb_i_write4_complete <= 1'b0; // Clear the flag when substate_pb_i_write4 is inactive
    end // end of if (substate_pb_i_write4_active) begin

*/
/************************************************************************************************************************/



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

/*
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
                   substate_pb_read4 <= SUBSTATE_PB_READ4_DONE;
               end
            end

            SUBSTATE_PB_READ4_DONE: begin
                // send response back
                hex_to_ascii(Read_Data_buffer, 4, ascii_out);
                uart_tx_string[0:10]  <={"p", "b", "_", "i", "_", "_", "r", "e", "a", "d", ","};
                uart_tx_string[11:20] <={ascii_out[0], ascii_out[1], ascii_out[2], ascii_out[3], 
                                         ascii_out[4], ascii_out[5], ascii_out[6], ascii_out[7], 8'h0D, 8'h0A};

                uart_tx_string_len  <= 21;
                uart_tx_process     <= 1'b1;   // Trigger UART transmission

                substate_pb_read4_complete <= 1'b1;   // Indicate substate_pb_read4 completion
                substate_pb_read4 <= SUBSTATE_PB_READ4_IDLE;
            end

        endcase
    end else begin
        substate_pb_read4_complete <= 1'b0; // Clear the flag when substate_pb_i_read4 is inactive
    end // end of if (substate_pb_i_read4_active) begin

*/
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

/********** Continuous Assignment **********/
// Assign bidirectional port with tristate buffer behavior


//assign DataPortPins = (data_dir) ? Data_Out_Port : 8'bz;  // drive Data_Out_Port if output
//assign Data_In_Port = DataPortPins;                      // read pins as input

endmodule