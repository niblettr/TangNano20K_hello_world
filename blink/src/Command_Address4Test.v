/*
;*****************************************************************************************
; function  asm_pb_i_address4_test    (test_adr_y) via (R7)
;*****************************************************************************************
; function:  test address y on lamp switch card 1,2,3,4
; The result is stored in TEST_CARD_Y_OUT[]
; parameters
; test_adr_y        defines test address from 0 to 7 (R7)
;*****************************************************************************************
_asm_pb_i_address4_test:
MOV     R1,#TEST_Y_READ       ; load startaddress from TEST_Y_READ[]
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
ADD     A,#BOARD_1 + TEST_ADDR_ON
MOV     P1,A                  ; P1=BOARD_1 + (8 * test_adr_y) + test_addr_on
NOP                           ; wait for data ready
NOP
MOVX    A,@R0                 ; read addresses via data bus
MOV     P1,#NO_BOARD_IDLE OR 0 OR CTR_OFF; reset Test_RD line
XRL     A,R3                  ; compare with 'XXX 00001B' with XXX=test_adr_y
MOV     @R1,A                 ; store Result(0 incidates o.k.)
INC     R1
ADD     A,#BOARD_2 + TEST_ADDR_ON
MOV     P1,A                  ; P1=BOARD_1 + (8 * test_adr_y) + test_addr_on
NOP                           ; wait for data ready
NOP
MOVX    A,@R0                 ; read addresses via data bus
MOV     P1,#NO_BOARD_IDLE OR 0 OR CTR_OFF; reset Test_RD line
XRL     A,R3                  ; compare with 'XXX 00001B' with XXX=test_adr_y
MOV     @R1,A                 ; store Result(0 incidates o.k.)
INC     R1
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
MOV     A,R4                  ; A=8 * test_adr_y
ADD     A,#BOARD_4 + TEST_ADDR_ON
RET
*/

/*
function asm_pb_i_address4_test(test_adr_y):
    
    result_ptr = address of TEST_Y_READ // Prepare result pointer and expected value
    expected_value = (test_adr_y << 5) | 0x01  // 'XXX00001' where XXX = test_adr_y

    // Prepare address offset for port1
    address_offset = test_adr_y << 3           // 8 * test_adr_y
    
    for board in [BOARD_1, BOARD_2, BOARD_3, BOARD_4]: // Test each board (1 to 4)
        // Set port1 to board address + offset + TEST_ADDR_ON
        port1 = board + address_offset + TEST_ADDR_ON
        
        wait_some_cycles()  // Wait for data to be ready (simulate with two NOPs)
        
        data = read_data_bus() // Read data from data bus
        
        port1 = NO_BOARD_IDLE | 0 | CTR_OFF // Reset Test_RD line
         
        result = data XOR expected_value // Compare read data with expected value

        store result at result_ptr // Store result (0 means OK)
        result_ptr = result_ptr + 1
    // End for
    return */

// >>>>>>>>>>>>. remeber, test_adr_y = command_param_data[0]

// P1 = Port1 :- 
    if (substate_pb_test_active) begin
        case (substate_pb_test)
            SUBSTATE_PB_TEST_IDLE: begin
                Board_ID_ptr    <= 0;
                data_dir <= DIR_INPUT;
                wait_multiples  <= 1;
                substate_pb_test_next <= SUBSTATE_PB_TEST_ADDR_ON;   
                substate_pb_test <= SUBSTATE_PB_TEST_WAIT_750N;
                end
            
            SUBSTATE_PB_TEST_ADDR_ON : begin //P1 = board + R4 + TEST_ADDR_ON
                BOARD_X <= BOARD_1 << Board_ID_ptr; //BOARD_X = 1, 2, 4 or 8
                TEST_ADDR_ON;
                wait_multiples <= 3; // total guess for time being...
                substate_pb_test <= SUBSTATE_PB_TEST_WAIT_750N;
                substate_pb_test_next <= SUBSTATE_TEST_READ_DATA;
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

            //data = read_data_bus() // Read address via data bus
            SUBSTATE_TEST_READ_DATA: begin
               Read_Data_buffer[Board_ID_ptr] <= Data_In_Port;
               wait_multiples <= 3; // guess
               substate_pb_test_next <= SUBSTATE_TEST_CTR_OFF;
               substate_pb_test <= SUBSTATE_PB_TEST_WAIT_750N;
            end
            
            SUBSTATE_TEST_CTR_OFF: begin //P1 = NO_BOARD_IDLE | 0 | CTR_OFF // Reset Test_RD line: P1 = NO_BOARD_IDLE | 0 | CTR_OFF
                CTR_OFF;
                NO_BOARD_IDLE;
                wait_multiples <= 3; // guess
                substate_pb_test_next <= SUBSTATE_PB_TEST_INC_CARD_ID_LOOP;
                substate_pb_test <= SUBSTATE_PB_TEST_WAIT_750N;
            end

            SUBSTATE_PB_TEST_INC_CARD_ID_LOOP: begin               
               if(Board_ID_ptr < (4 - 1)) begin   // 0->3 is 4 hence the -1  
                  Board_ID_ptr <= Board_ID_ptr + 3'd1; 
                  substate_pb_test <= SUBSTATE_PB_TEST_ADDR_ON; // loop back round to do remaining cards
               end else begin
                   substate_pb_test <= SUBSTATE_PB_TEST_DONE;
               end
            end

            SUBSTATE_PB_TEST_DONE: begin
               substate_pb_test_complete <= 1'b1;
               substate_pb_test <= SUBSTATE_PB_TEST_IDLE;
            end

        endcase
    end else begin
        substate_pb_test_complete <= 1'b0; // Clear the flag when substate_pb_i_read4 is inactive
    end // end of if (substate_pb_i_read4_active) begin