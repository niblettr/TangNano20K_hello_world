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
                  debug_hex_reg_test =  Board_ID_ptr;    // niblett   re-enable
                  Board_ID_ptr <= Board_ID_ptr + 3'd1; 
                  substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_ASSERT_ADDRESS_ID; // loop back round to do remaining cards
               end else begin
                   substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_DONE;
               end
            end

            SUBSTATE_PB_I_WRITE4_DONE: begin
                //send_debug_message(debug_hex_reg_test, {"W", "r", "i", "t", "e", " ", "0", "x"}, 8);
                substate_pb_i_write4_complete <= 1'b1;   // Indicate substate_pb_i_write4 completion
                substate_pb_i_write4 <= SUBSTATE_PB_I_WRITE4_IDLE;
            end
        endcase
    end else begin
        substate_pb_i_write4_complete <= 1'b0; // Clear the flag when substate_pb_i_write4 is inactive // niblett re-enable
    end // end of if (substate_pb_i_write4_active) begin


/************************************************************************************************************************/