module state_machines #(
    parameter CLOCK_FREQUENCY = 27000000 // System clock frequency in Hz
)(
    input       clock,    // System clock
    input       reset,

    // Activation Inputs
    input       reg       substate_pb_i_write4_active,
    input       reg       substate_pb_read4_active,
    input       reg       substate_pb_adc4_active,
    input       reg       substate_pb_adc1_active,
    input       reg       substate_pb_test_active,

    // Completion Outputs
    output      reg       substate_pb_i_write4_complete, 
    output      reg       substate_pb_read4_complete,
    output      reg       substate_pb_adc4_complete,
    output      reg       substate_pb_adc1_complete, 
    output      reg       substate_pb_test_complete, 

    // Board Control Outputs
    output      reg [3:0] BOARD_X,
    output      reg [2:0] AddessPortPin,
    output      reg       TestAddressP,
    output      reg       RdP,
    output      reg       WrP,
    output      reg       LampResetPin,

    // Command Parameters
    input       reg [7:0] command_param_data [0:3],
    input       reg [1:0] CommandType,

    // Data Ports
    output      reg [7:0] Data_Out_Port,
    input       [7:0]     Data_In_Port,
    output      reg       data_dir,

    // Response Handling
    output       reg [7:0] ResponseBytes[0:3],
    output       reg [3:0] ResponseByteCount
);

`include "utils.v"

/*********************************************************************************************************/
parameter PORT_MUX = 3'h18; // 0x18 // --011--- /  Same Port address

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
    SUBSTATE_PB_READ4_RESPOND,
    SUBSTATE_PB_READ4_DONE              
} substate_pb_read4_t;
substate_pb_read4_t substate_pb_read4            = SUBSTATE_PB_READ4_IDLE;
substate_pb_read4_t substate_pb_read4_next       = SUBSTATE_PB_READ4_IDLE;

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

substate_pb_adc4_t substate_pb_adc4              = SUBSTATE_PB_ADC4_IDLE;
substate_pb_adc4_t substate_pb_adc4_next         = SUBSTATE_PB_ADC4_IDLE;

typedef enum logic [3:0] {
    SUBSTATE_PB_ADC1_IDLE              = 4'b0000,
    SUBSTATE_PB_ADC1_PRE_DELAY,
    SUBSTATE_PB_ADC1_ASSERT_ADDRESS_ID,
    SUBSTATE_PB_ADC1_WAIT_750N,
    SUBSTATE_PB_ADC1_ASSERT_DATA,
    SUBSTATE_PB_ADC1_ASSERT_WR_ENABLE,
    SUBSTATE_PB_ADC1_RELEASE_WR_ENABLE,
    SUBSTATE_PB_ADC1_RELEASE_DATA,
    SUBSTATE_PB_ADC1_INC_CARD_ID_LOOP,
    SUBSTATE_PB_ADC1_TEST_READ,
    SUBSTATE_PB_ADC1_DONE              
} substate_pb_adc1_t;

substate_pb_adc1_t substate_pb_adc1              = SUBSTATE_PB_ADC1_IDLE;
substate_pb_adc1_t substate_pb_adc1_next         = SUBSTATE_PB_ADC1_IDLE;

typedef enum logic [3:0] {
    SUBSTATE_PB_TEST_IDLE              = 4'b0000,
    SUBSTATE_PB_TEST_PRE_DELAY,
    SUBSTATE_PB_TEST_ASSERT_ADDRESS_ID,
    SUBSTATE_PB_TEST_WAIT_750N,
    SUBSTATE_PB_TEST_ASSERT_DATA,
    SUBSTATE_PB_TEST_ASSERT_WR_ENABLE,
    SUBSTATE_PB_TEST_RELEASE_WR_ENABLE,
    SUBSTATE_PB_TEST_RELEASE_DATA,
    SUBSTATE_PB_TEST_INC_CARD_ID_LOOP,
    SUBSTATE_PB_TEST_TEST_READ,
    SUBSTATE_PB_TEST_DONE              
} substate_pb_test_t;

substate_pb_test_t substate_pb_test              = SUBSTATE_PB_TEST_IDLE;
substate_pb_test_t substate_pb_test_next         = SUBSTATE_PB_TEST_IDLE;

reg [4:0] substate_wait_counter  = 1; // NOTE: set to 1 instead of 0 because statemachine transitions waste 1 clock cyle
reg [4:0] wait_multiples         = 0; // 4bit : 0>31
reg [2:0] Board_ID_ptr           = 0;

reg [7:0] Read_Data_buffer [4];

reg [7:0] uart_tx_response_string [0:20];
reg [7:0] uart_tx_response_string_len;
reg       uart_tx_response_process = 1'b0;

reg [7:0] reset_counter    = 8'b0; // 1-bit counter for reset delay

always @(posedge clock) begin


    `include "InitCard.v"       // code for the Card initialisation

    `include "Command_Read4.v" // code/statmachine for pb_i_read

    `include "Command_Write4.v" // code/statmachine for pb_i_write


/************************************************************************************************************************/

/*
;*****************************************************************************************
; function  asm_pb_i_address4_test    (test_adr_y) via (R7)
;*****************************************************************************************
; function:  test address y on lamp switch card 1,2,3,4
; The result is stored in TEST_CARD_Y_OUT[]
; parameters
; test_adr_y        defines test address from 0 to 7 (R7)
;*****************************************************************************************
>>>>>>>>>>>>>>>>>>>>>>>>>>>     _asm_pb_i_address4_test:
MOV     R1,#TEST_Y_READ       ; load startaddress from TEST_Y_READ[]

; prepare compare-value
; ---------------------
; XXX=TEST_Y
MOV     A,R7                  ; A=00000XXX
SWAP    A                     ; A=0XXX0000
RL      A                     ; A=XXX00000
SETB    ACC.0                 ; A=XXX00001
MOV     R3,A                  ; R3=(32 * TEST_Y) + 1 or XXX00001

MOV     A,R7                  ; prepare address port1 value
RL      A
RL      A
RL      A
MOV     R4,A                  ; R4=8*test_adr_y=00XXX000

;ST800: P1 = 00XXXBBB where 00=TEST_ADDR_ON, XXX=TEST_Y(0-7) and BBB=000=BOARD_1
ADD     A,#BOARD_1 + TEST_ADDR_ON
MOV     P1,A                  ; P1=BOARD_1 + (8 * test_adr_y) + test_addr_on
NOP                           ; wait for data ready
NOP
MOVX    A,@R0                 ; read addresses via data bus
MOV     P1,#NO_BOARD_IDLE OR 0 OR CTR_OFF; reset Test_RD line

XRL     A,R3                  ; compare with 'XXX 00001B' with XXX=test_adr_y
MOV     @R1,A                 ; store Result(0 incidates o.k.)
INC     R1

;ST800: P1 = 00XXXBBB where 00=TEST_ADDR_ON, XXX=TEST_Y(0-7) and BBB=001=BOARD_2
MOV     A,R4                  ; A=8 * test_adr_y

ADD     A,#BOARD_2 + TEST_ADDR_ON
MOV     P1,A                  ; P1=BOARD_1 + (8 * test_adr_y) + test_addr_on
NOP                           ; wait for data ready
NOP
MOVX    A,@R0                 ; read addresses via data bus
MOV     P1,#NO_BOARD_IDLE OR 0 OR CTR_OFF; reset Test_RD line

XRL     A,R3                  ; compare with 'XXX 00001B' with XXX=test_adr_y
MOV     @R1,A                 ; store Result(0 incidates o.k.)
INC     R1

;ST800: P1 = 00XXXBBB where 00=TEST_ADDR_ON, XXX=TEST_Y(0-7) and BBB=010=BOARD_3
MOV     A,R4                  ; A=8 * test_adr_y

ADD     A,#BOARD_3 + TEST_ADDR_ON

MOV     P1,A                  ; P1=BOARD_1 + (8 * test_adr_y) + test_addr_on
NOP                           ; wait for data ready
NOP
MOVX    A,@R0                 ; read addresses via data bus
MOV     P1,#NO_BOARD_IDLE OR 0 OR CTR_OFF; reset Test_RD line

XRL     A,R3                  ; compare with 'XXX 00001B' with XXX=test_adr_y
MOV     @R1,A                 ; store Result(0 incidates o.k.)
INC     R1

;ST800: P1 = 00XXXBBB where 00=TEST_ADDR_ON, XXX=TEST_Y(0-7) and BBB=011=BOARD_4
MOV     A,R4                  ; A=8 * test_adr_y

ADD     A,#BOARD_4 + TEST_ADDR_ON

MOV     P1,A                  ; P1=BOARD_1 + (8 * test_adr_y) + test_addr_on
NOP                           ; wait for data ready
NOP
MOVX    A,@R0                 ; read addresses via data bus
MOV     P1,#NO_BOARD_IDLE OR 0 OR CTR_OFF; reset Test_RD line

XRL     A,R3                  ; compare with 'XXX 00001B' with XXX=test_adr_y
MOV     @R1,A                 ; store Result(0 incidates o.k.)

RET
*/


/*
///////////////////////////////////////////////////////////////////////////////////////////////
// Pseudocode for asm_pb_i_address4_test(test_adr_y)
// Purpose: Test address 'test_adr_y' on lamp switch cards 1-4
// Result is stored in TEST_CARD_Y_OUT[] array

function asm_pb_i_address4_test(test_adr_y):
    R1 = address of TEST_Y_READ      // Output/result pointer    
    R4 = test_adr_y << 3 // Prepare address port1 value: R4 = 8 * test_adr_y

    for board in [BOARD_1, BOARD_2, BOARD_3, BOARD_4]:
        
        P1 = board + R4 + TEST_ADDR_ON // Set P1 = board + (8 * test_adr_y) + TEST_ADDR_ON        
        wait_some_cycles() // Wait for data ready (simulate with two NOPs)        
        data = read_data_bus() // Read address via data bus        
        P1 = NO_BOARD_IDLE | 0 | CTR_OFF // Reset Test_RD line: P1 = NO_BOARD_IDLE | 0 | CTR_OFF        
        store_result(R1, result) // Store result (0 means OK) in TEST_Y_READ[]
        R1 = R1 + 1
    // End for

    return
*/


    if (substate_pb_test_active) begin
        case (substate_pb_test)
            SUBSTATE_PB_TEST_IDLE: begin
                Board_ID_ptr    <= 0;
                data_dir        <= data_dir <= DIR_INPUT;
                wait_multiples  <= 1;
                substate_pb_test_next = SUBSTATE_PB_TEST_DONE;
                substate_pb_test <= SUBSTATE_PB_TEST_WAIT_750N;
                end

            //MOV     P1,#BOARD_ALL OR PORT_MUX OR CTR_OFF
            //#define BOARD_ALL            0x05 // -----101
            SUBSTATE_PB_TEST_ASSERT_ADDRESS_ID: begin
                BOARD_X <= 5;//BOARD_ALL;
                AddessPortPin <= PORT_MUX;
                WrP <= DISABLE; // CTR_OFF in the assembler
                RdP <= DISABLE; // CTR_OFF in the assembler
                wait_multiples <= 1;
                substate_pb_test <= SUBSTATE_PB_TEST_WAIT_750N;
                substate_pb_test_next <= SUBSTATE_PB_TEST_DONE; // <<<<<<<<<<<<<<<<<<<<<<
            end

            SUBSTATE_PB_TEST_WAIT_750N: begin
                if(wait_multiples) begin
                    if (substate_wait_counter < 21) begin // Proceed after 21 cycles (~777ns) if clock = 20MHZ, 750ns can be achieved
                        substate_wait_counter <= substate_wait_counter + 1'b1;
                    end else begin                        
                        substate_wait_counter <= 0;
                        wait_multiples <= wait_multiples - 3'd1;
                    end
                end else begin
                   substate_pb_test <= substate_pb_test_next;
                end
            end
            SUBSTATE_PB_TEST_DONE: begin
                //if(CommandType == 0) begin
                   ResponseBytes[0:3] <= Read_Data_buffer[0:3];
                   ResponseByteCount  <= 4;
                //end

                substate_pb_test_complete <= 1'b1;
                substate_pb_test <= SUBSTATE_PB_TEST_IDLE;
            end

        endcase
    end else begin
        substate_pb_test_complete <= 1'b0; // Clear the flag when substate_pb_i_read4 is inactive
    end // end of if (substate_pb_i_read4_active) begin




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

    // handles the pb_adc4_16 & pb_adc4_8
    // doing pb_adc4_16 first
    if (substate_pb_adc4_active) begin
        case (substate_pb_adc4)
            SUBSTATE_PB_ADC4_IDLE: begin
                Board_ID_ptr <= 0;
                data_dir <= DIR_INPUT;
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
                //if(CommandType == 0) begin
                   ResponseBytes[0:3] <= Read_Data_buffer[0:3];
                   ResponseByteCount  <= 4;
                //end

                substate_pb_adc4_complete <= 1'b1;   // Indicate substate_pb_adc4 completion
                substate_pb_adc4 <= SUBSTATE_PB_ADC4_IDLE;
            end

        endcase
    end else begin
        substate_pb_adc4_complete <= 1'b0; // Clear the flag when substate_pb_i_read4 is inactive
    end // end of if (substate_pb_i_read4_active) begin














    if (substate_pb_adc1_active) begin
        case (substate_pb_adc1)
            SUBSTATE_PB_ADC1_IDLE: begin
                Board_ID_ptr <= 0;
                data_dir <= DIR_INPUT;
                wait_multiples <= 3;
                substate_pb_adc1_next = SUBSTATE_PB_ADC1_DONE;
                substate_pb_adc1 <= SUBSTATE_PB_ADC1_WAIT_750N;
                end

            //MOV     P1,#BOARD_ALL OR PORT_MUX OR CTR_OFF
            //#define BOARD_ALL            0x05 // -----101
            SUBSTATE_PB_ADC1_ASSERT_ADDRESS_ID: begin
                BOARD_X <= 5;//BOARD_ALL;
                AddessPortPin <= PORT_MUX;
                WrP <= DISABLE; // CTR_OFF in the assembler
                RdP <= DISABLE; // CTR_OFF in the assembler
                wait_multiples <= 1;
                substate_pb_adc1 <= SUBSTATE_PB_ADC1_WAIT_750N;
                substate_pb_adc1_next <= SUBSTATE_PB_ADC1_DONE; // <<<<<<<<<<<<<<<<<<<<<<
            end

            SUBSTATE_PB_ADC1_WAIT_750N: begin
                if(wait_multiples) begin
                    if (substate_wait_counter < 21) begin // Proceed after 21 cycles (~777ns) if clock = 20MHZ, 750ns can be achieved
                        substate_wait_counter <= substate_wait_counter + 1'b1;
                    end else begin                        
                        substate_wait_counter <= 0;
                        wait_multiples <= wait_multiples - 3'd1;
                    end
                end else begin
                   substate_pb_adc1 <= substate_pb_adc1_next;
                end
            end
            SUBSTATE_PB_ADC4_DONE: begin
                //if(CommandType == 0) begin
                   ResponseBytes[0:3] <= Read_Data_buffer[0:3];
                   ResponseByteCount  <= 4;
                //end

                substate_pb_adc1_complete <= 1'b1;   // Indicate substate_pb_adc4 completion
                substate_pb_adc1 <= SUBSTATE_PB_ADC1_IDLE;
            end

        endcase
    end else begin
        substate_pb_adc1_complete <= 1'b0; // Clear the flag when substate_pb_i_read4 is inactive
    end // end of if (substate_pb_i_read4_active) begin




end
endmodule